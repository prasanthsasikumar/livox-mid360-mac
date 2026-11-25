import Foundation
import Network

/// A very small HTTP server so the computer running the lidar can drive the phone.
///
///   GET  /status                 what the camera is doing, free space, the phone's clock
///   POST /start?name=walk-1      lock exposure/focus/white balance and start recording
///        optional: lens=wide|ultrawide, shutter=0.005 (longest exposure per frame, seconds),
///        focus=lock|auto (default: lock on the ultra-wide lens, keep focusing on the main lens)
///   POST /stop                   stop and report the file
///   GET  /clips                  list recorded files
///   GET  /clips/<file>           download a file
///   POST /delete?file=<file>     remove a file from the phone
///
/// Advertised on the local network as "_lidarcam._tcp" so the computer can find it by name.
final class Server: ObservableObject {
    static let port: UInt16 = 8765
    private var listener: NWListener?
    private let queue = DispatchQueue(label: "lidarcam.server")
    private let recorder: Recorder
    @Published var address = "starting…"

    init(recorder: Recorder) { self.recorder = recorder }

    func start() {
        guard let listener = try? NWListener(using: .tcp, on: NWEndpoint.Port(rawValue: Server.port)!) else {
            address = "could not open port \(Server.port)"
            return
        }
        listener.service = NWListener.Service(name: "LidarCam", type: "_lidarcam._tcp")
        listener.newConnectionHandler = { [weak self] connection in
            connection.start(queue: self?.queue ?? .main)
            self?.read(connection, buffer: Data())
        }
        listener.stateUpdateHandler = { [weak self] state in
            DispatchQueue.main.async {
                switch state {
                case .ready: self?.address = "\(Server.localAddress() ?? "?"):\(Server.port)"
                case .failed(let error): self?.address = "server failed: \(error.localizedDescription)"
                default: break
                }
            }
        }
        listener.start(queue: queue)
        self.listener = listener
    }

    private func read(_ connection: NWConnection, buffer: Data) {
        connection.receive(minimumIncompleteLength: 1, maximumLength: 65536) { [weak self] data, _, done, error in
            guard let self else { return }
            var buffer = buffer
            if let data { buffer.append(data) }
            if let end = buffer.range(of: Data("\r\n\r\n".utf8)) {
                let head = String(decoding: buffer[..<end.lowerBound], as: UTF8.self)
                self.route(head.components(separatedBy: "\r\n").first ?? "", on: connection)
            } else if done || error != nil || buffer.count > 65536 {
                connection.cancel()
            } else {
                self.read(connection, buffer: buffer)
            }
        }
    }

    private func route(_ requestLine: String, on connection: NWConnection) {
        let parts = requestLine.split(separator: " ")
        guard parts.count >= 2, let url = URLComponents(string: String(parts[1])) else {
            return send(connection, status: "400 Bad Request", json: ["error": "bad request"])
        }
        let method = String(parts[0])
        var query: [String: String] = [:]
        url.queryItems?.forEach { query[$0.name] = $0.value ?? "" }
        let reply: ([String: Any]) -> Void = { [weak self] body in
            self?.send(connection, status: body["error"] == nil ? "200 OK" : "409 Conflict", json: body)
        }

        switch (method, url.path) {
        case ("GET", "/status"):
            reply(recorder.snapshot())
        case ("POST", "/start"):
            let name = Server.safe(query["name"] ?? "clip-\(Int(Date().timeIntervalSince1970))")
            recorder.start(name: name, lens: query["lens"], maxShutter: Double(query["shutter"] ?? "") ?? 0.005,
                           focus: query["focus"], reply: reply)
        case ("POST", "/stop"):
            recorder.stop(reply: reply)
        case ("GET", "/clips"):
            let files = (try? FileManager.default.contentsOfDirectory(atPath: Recorder.documents.path)) ?? []
            reply(["files": files.sorted()])
        case ("POST", "/delete"):
            let file = Recorder.documents.appendingPathComponent(Server.safe(query["file"] ?? ""))
            do { try FileManager.default.removeItem(at: file); reply(["deleted": file.lastPathComponent]) }
            catch { reply(["error": error.localizedDescription]) }
        case ("GET", let path) where path.hasPrefix("/clips/"):
            let name = Server.safe(String(path.dropFirst("/clips/".count)).removingPercentEncoding ?? "")
            sendFile(Recorder.documents.appendingPathComponent(name), on: connection)
        default:
            send(connection, status: "404 Not Found", json: ["error": "unknown path"])
        }
    }

    /// File names only: no folders, nothing that could climb out of Documents.
    private static func safe(_ name: String) -> String {
        String(name.filter { $0.isLetter || $0.isNumber || "-_.".contains($0) }.drop(while: { $0 == "." }))
    }

    private func send(_ connection: NWConnection, status: String, json: [String: Any]) {
        let body = (try? JSONSerialization.data(withJSONObject: json)) ?? Data("{}".utf8)
        var message = Data("HTTP/1.1 \(status)\r\nContent-Type: application/json\r\nContent-Length: \(body.count)\r\nConnection: close\r\n\r\n".utf8)
        message.append(body)
        connection.send(content: message, completion: .contentProcessed { _ in connection.cancel() })
    }

    private func sendFile(_ url: URL, on connection: NWConnection) {
        guard let handle = try? FileHandle(forReadingFrom: url),
              let size = (try? FileManager.default.attributesOfItem(atPath: url.path)[.size] as? NSNumber)?.int64Value else {
            return send(connection, status: "404 Not Found", json: ["error": "no such file"])
        }
        let head = "HTTP/1.1 200 OK\r\nContent-Type: application/octet-stream\r\nContent-Length: \(size)\r\nConnection: close\r\n\r\n"
        connection.send(content: Data(head.utf8), completion: .contentProcessed { [weak self] error in
            if error != nil { try? handle.close(); connection.cancel(); return }
            self?.sendChunk(handle, on: connection)
        })
    }

    private func sendChunk(_ handle: FileHandle, on connection: NWConnection) {
        let chunk = (try? handle.read(upToCount: 1 << 20)) ?? Data()
        if chunk.isEmpty {
            try? handle.close()
            connection.send(content: nil, isComplete: true, completion: .contentProcessed { _ in connection.cancel() })
            return
        }
        connection.send(content: chunk, completion: .contentProcessed { [weak self] error in
            if error != nil { try? handle.close(); connection.cancel(); return }
            self?.sendChunk(handle, on: connection)
        })
    }

    /// The phone's Wi-Fi IPv4 address, for showing on screen.
    static func localAddress() -> String? {
        var result: String?
        var list: UnsafeMutablePointer<ifaddrs>?
        guard getifaddrs(&list) == 0 else { return nil }
        defer { freeifaddrs(list) }
        var cursor = list
        while let entry = cursor {
            let interface = entry.pointee
            if interface.ifa_addr.pointee.sa_family == UInt8(AF_INET), String(cString: interface.ifa_name) == "en0" {
                var host = [CChar](repeating: 0, count: Int(NI_MAXHOST))
                getnameinfo(interface.ifa_addr, socklen_t(interface.ifa_addr.pointee.sa_len), &host, socklen_t(host.count),
                            nil, 0, NI_NUMERICHOST)
                result = String(cString: host)
            }
            cursor = interface.ifa_next
        }
        return result
    }
}
