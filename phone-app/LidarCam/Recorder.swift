import AVFoundation
import CoreMotion
import UIKit

/// Owns the camera. Records 4K video to the app's Documents folder with exposure,
/// focus and white balance locked, so colours do not drift during a scan.
final class Recorder: NSObject, ObservableObject, AVCaptureFileOutputRecordingDelegate {
    let session = AVCaptureSession()
    private let output = AVCaptureMovieFileOutput()
    private let queue = DispatchQueue(label: "lidarcam.camera")
    private let motion = CMMotionManager()
    private var device: AVCaptureDevice?
    private var pendingStart: (([String: Any]) -> Void)?
    private var pendingStop: (([String: Any]) -> Void)?
    private var startInfo: [String: Any] = [:]

    @Published var status = "Starting camera…"
    @Published var isRecording = false
    @Published var lens = "wide"

    static var documents: URL {
        FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
    }

    func boot() {
        AVCaptureDevice.requestAccess(for: .video) { granted in
            guard granted else {
                DispatchQueue.main.async { self.status = "Camera access was refused. Allow it in Settings." }
                return
            }
            self.queue.async { self.configure(lens: "wide") }
        }
        motion.accelerometerUpdateInterval = 0.1
        motion.startAccelerometerUpdates()
    }

    /// lens: "wide" (1x, sharpest) or "ultrawide" (0.5x, sees more).
    private func configure(lens name: String) {
        let type: AVCaptureDevice.DeviceType = name == "ultrawide" ? .builtInUltraWideCamera : .builtInWideAngleCamera
        guard let camera = AVCaptureDevice.default(type, for: .video, position: .back),
              let input = try? AVCaptureDeviceInput(device: camera) else {
            DispatchQueue.main.async { self.status = "No \(name) camera found." }
            return
        }
        session.beginConfiguration()
        session.inputs.forEach { session.removeInput($0) }
        session.sessionPreset = .hd4K3840x2160
        if session.canAddInput(input) { session.addInput(input) }
        if !session.outputs.contains(output), session.canAddOutput(output) { session.addOutput(output) }
        if let connection = output.connection(with: .video) {
            // Stabilisation crops and warps the picture, which breaks 3D reconstruction.
            if connection.isVideoStabilizationSupported { connection.preferredVideoStabilizationMode = .off }
            if output.availableVideoCodecTypes.contains(.hevc) {
                output.setOutputSettings([AVVideoCodecKey: AVVideoCodecType.hevc], for: connection)
            }
        }
        session.commitConfiguration()
        device = camera
        unlock()
        if !session.isRunning { session.startRunning() }
        DispatchQueue.main.async {
            self.lens = name
            self.status = "Ready"
        }
    }

    private func unlock() {
        guard let camera = device, (try? camera.lockForConfiguration()) != nil else { return }
        if camera.isFocusModeSupported(.continuousAutoFocus) { camera.focusMode = .continuousAutoFocus }
        if camera.isExposureModeSupported(.continuousAutoExposure) { camera.exposureMode = .continuousAutoExposure }
        if camera.isWhiteBalanceModeSupported(.continuousAutoWhiteBalance) { camera.whiteBalanceMode = .continuousAutoWhiteBalance }
        camera.unlockForConfiguration()
    }

    /// Freeze focus and white balance where they are, and trade exposure time for ISO so
    /// each frame is exposed for at most `maxShutter` seconds (less motion blur while walking).
    ///
    /// Focus: the ultra-wide lens is sharp from about half a metre to infinity, so it is locked.
    /// The main lens has a shallow depth of field indoors, so by default it keeps focusing;
    /// locking it risks freezing on whatever was nearest when recording began.
    private func lock(maxShutter: Double, lockFocus: Bool) -> [String: Any] {
        guard let camera = device, (try? camera.lockForConfiguration()) != nil else { return [:] }
        defer { camera.unlockForConfiguration() }
        if lockFocus, camera.isFocusModeSupported(.locked) { camera.focusMode = .locked }
        if camera.isWhiteBalanceModeSupported(.locked) { camera.whiteBalanceMode = .locked }
        var shutter = camera.exposureDuration.seconds
        var iso = Double(camera.iso)
        if camera.isExposureModeSupported(.custom) {
            let format = camera.activeFormat
            if shutter > maxShutter {
                let wanted = iso * shutter / maxShutter
                let allowed = min(wanted, Double(format.maxISO))
                shutter = iso * shutter / allowed          // if ISO runs out, keep brightness and accept a longer shutter
                iso = allowed
            }
            iso = max(iso, Double(format.minISO))
            let duration = CMTime(seconds: shutter, preferredTimescale: 1_000_000)
            camera.setExposureModeCustom(duration: duration, iso: Float(iso), completionHandler: nil)
        } else if camera.isExposureModeSupported(.locked) {
            camera.exposureMode = .locked
        }
        return ["shutter_s": shutter, "iso": iso, "lens_position": camera.lensPosition, "focus_locked": lockFocus]
    }

    func start(name: String, lens wanted: String?, maxShutter: Double, focus: String?, reply: @escaping ([String: Any]) -> Void) {
        queue.async {
            if self.output.isRecording { reply(["error": "already recording"]); return }
            if let wanted, wanted != self.lens { self.configure(lens: wanted); Thread.sleep(forTimeInterval: 1.5) }
            guard self.session.isRunning, self.device != nil else { reply(["error": "camera not ready"]); return }
            let url = Recorder.documents.appendingPathComponent(name + ".mov")
            try? FileManager.default.removeItem(at: url)
            let lockFocus = focus == "lock" || (focus != "auto" && self.lens == "ultrawide")
            var info = self.lock(maxShutter: maxShutter, lockFocus: lockFocus)
            if let g = self.motion.accelerometerData?.acceleration {
                info["gravity"] = [g.x, g.y, g.z]
                // Tag the video with whichever way up the phone is mounted, so frames come out upright.
                // Angle 0 is the sensor's own orientation: landscape with the phone's left edge up (x reads -1 g).
                let angle: CGFloat = abs(g.x) > abs(g.y) ? (g.x > 0 ? 180 : 0) : (g.y < 0 ? 90 : 270)
                if let connection = self.output.connection(with: .video), connection.isVideoRotationAngleSupported(angle) {
                    connection.videoRotationAngle = angle
                    info["rotation"] = angle
                }
            }
            info["name"] = name
            info["lens"] = self.lens
            self.startInfo = info
            self.pendingStart = reply
            self.output.startRecording(to: url, recordingDelegate: self)
        }
    }

    func stop(reply: @escaping ([String: Any]) -> Void) {
        queue.async {
            guard self.output.isRecording else { reply(["error": "not recording"]); return }
            self.pendingStop = reply
            self.output.stopRecording()
        }
    }

    func snapshot() -> [String: Any] {
        let free = (try? Recorder.documents.resourceValues(forKeys: [.volumeAvailableCapacityForImportantUsageKey]))?
            .volumeAvailableCapacityForImportantUsage ?? 0
        UIDevice.current.isBatteryMonitoringEnabled = true
        return ["recording": output.isRecording, "lens": lens, "ready": session.isRunning,
                "seconds": output.recordedDuration.seconds.isFinite ? output.recordedDuration.seconds : 0,
                "free_gb": Double(free) / 1e9, "battery": UIDevice.current.batteryLevel,
                "phone_time": Date().timeIntervalSince1970]
    }

    // The time the first frame was captured, on the phone's wall clock.
    private func started(at firstFrame: Date) {
        var info = startInfo
        info["first_frame_time"] = firstFrame.timeIntervalSince1970
        pendingStart?(info)
        pendingStart = nil
        DispatchQueue.main.async { self.isRecording = true; self.status = "Recording \(info["name"] ?? "")" }
    }

    func fileOutput(_ output: AVCaptureFileOutput, didStartRecordingTo fileURL: URL, startPTS: CMTime,
                    from connections: [AVCaptureConnection]) {
        // startPTS is on the host clock; convert it to wall-clock time.
        let age = CMClockGetTime(CMClockGetHostTimeClock()).seconds - startPTS.seconds
        started(at: Date().addingTimeInterval(-age))
    }

    func fileOutput(_ output: AVCaptureFileOutput, didStartRecordingTo fileURL: URL, from connections: [AVCaptureConnection]) {
        if pendingStart != nil { started(at: Date()) }
    }

    func fileOutput(_ output: AVCaptureFileOutput, didFinishRecordingTo outputFileURL: URL,
                    from connections: [AVCaptureConnection], error: Error?) {
        let size = (try? FileManager.default.attributesOfItem(atPath: outputFileURL.path)[.size] as? NSNumber)?.int64Value ?? 0
        var result: [String: Any] = ["file": outputFileURL.lastPathComponent, "bytes": size,
                                     "seconds": output.recordedDuration.seconds]
        if let error = error as NSError?,
           (error.userInfo[AVErrorRecordingSuccessfullyFinishedKey] as? Bool) != true {
            result["error"] = error.localizedDescription
        }
        unlock()
        pendingStop?(result)
        pendingStop = nil
        DispatchQueue.main.async {
            self.isRecording = false
            self.status = String(format: "Saved %@ (%.0f MB)", outputFileURL.lastPathComponent, Double(size) / 1e6)
        }
    }
}
