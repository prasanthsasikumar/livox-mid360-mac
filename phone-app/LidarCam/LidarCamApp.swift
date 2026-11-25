import AVFoundation
import SwiftUI

@main
struct LidarCamApp: App {
    @StateObject private var recorder: Recorder
    @StateObject private var server: Server

    init() {
        let recorder = Recorder()
        _recorder = StateObject(wrappedValue: recorder)
        _server = StateObject(wrappedValue: Server(recorder: recorder))
    }

    var body: some Scene {
        WindowGroup {
            ContentView(recorder: recorder, server: server)
                .onAppear {
                    UIApplication.shared.isIdleTimerDisabled = true   // never dim or lock while mounted on the rig
                    recorder.boot()
                    server.start()
                }
        }
    }
}

struct ContentView: View {
    @ObservedObject var recorder: Recorder
    @ObservedObject var server: Server

    var body: some View {
        ZStack(alignment: .bottomLeading) {
            Preview(session: recorder.session).ignoresSafeArea()
            VStack(alignment: .leading, spacing: 6) {
                HStack(spacing: 8) {
                    Circle().fill(recorder.isRecording ? Color.red : Color.green).frame(width: 14, height: 14)
                    Text(recorder.status).font(.headline)
                }
                Text("Lens: \(recorder.lens) · 4K · controlled from the computer")
                Text("Address: \(server.address)").font(.system(.body, design: .monospaced))
            }
            .foregroundStyle(.white)
            .padding(12)
            .background(.black.opacity(0.55), in: RoundedRectangle(cornerRadius: 12))
            .padding(16)
        }
        .background(Color.black)
    }
}

/// Live camera picture.
struct Preview: UIViewRepresentable {
    let session: AVCaptureSession

    final class View: UIView {
        override class var layerClass: AnyClass { AVCaptureVideoPreviewLayer.self }
        var preview: AVCaptureVideoPreviewLayer { layer as! AVCaptureVideoPreviewLayer }
    }

    func makeUIView(context: Context) -> View {
        let view = View()
        view.preview.session = session
        view.preview.videoGravity = .resizeAspect
        return view
    }

    func updateUIView(_ view: View, context: Context) {}
}
