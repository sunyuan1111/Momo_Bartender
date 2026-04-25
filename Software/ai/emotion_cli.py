#!/usr/bin/env python3

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2

from common import (
    DEFAULT_EMOTION_ENGINE,
    DEFAULT_EMOTION_MODEL,
    RUNS_ROOT,
    ensure_runtime_dirs,
    preload_cuda_user_libs,
    save_demo_face,
    torch_device,
    write_json,
)


def _build_face_detector():
    cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    detector = cv2.CascadeClassifier(cascade_path)
    if detector.empty():
        raise SystemExit(f"Failed to load Haar cascade: {cascade_path}")
    return detector


def _detect_primary_face(image_bgr, detector):
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    faces = detector.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(48, 48))
    if len(faces) == 0:
        return image_bgr, None
    x, y, w, h = max(faces, key=lambda box: box[2] * box[3])
    return image_bgr[y : y + h, x : x + w], [int(x), int(y), int(w), int(h)]


def _predict_emotion(image_bgr, detector, recognizer, device: str):
    face_bgr, face_bbox = _detect_primary_face(image_bgr, detector)
    face_rgb = cv2.cvtColor(face_bgr, cv2.COLOR_BGR2RGB)
    labels, scores = recognizer.predict_emotions(face_rgb, logits=False)
    label = labels[0]
    score_map = {
        recognizer.idx_to_emotion_class[idx]: float(scores[0][idx])
        for idx in range(min(len(scores[0]), len(recognizer.idx_to_emotion_class)))
    }
    return {
        "face_bbox": face_bbox,
        "device": device,
        "label": label,
        "scores": score_map,
    }


def _draw_overlay(frame_bgr, payload, fps: float | None):
    annotated = frame_bgr.copy()
    face_bbox = payload.get("face_bbox")
    label = payload.get("label", "Unknown")
    scores = payload.get("scores", {})
    confidence = float(scores.get(label, 0.0))

    if face_bbox is not None:
        x, y, w, h = face_bbox
        cv2.rectangle(annotated, (x, y), (x + w, y + h), (30, 220, 120), 2)
        cv2.putText(
            annotated,
            f"{label} {confidence:.2f}",
            (x, max(30, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (30, 220, 120),
            2,
            cv2.LINE_AA,
        )
    else:
        cv2.putText(
            annotated,
            "No face detected",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (40, 120, 255),
            2,
            cv2.LINE_AA,
        )

    if fps is not None:
        cv2.putText(
            annotated,
            f"FPS {fps:.1f}",
            (20, annotated.shape[0] - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 200, 50),
            2,
            cv2.LINE_AA,
        )

    return annotated


def _open_camera(camera_index: int, width: int | None, height: int | None):
    tried = []
    for backend in (cv2.CAP_V4L2, cv2.CAP_ANY):
        cap = cv2.VideoCapture(camera_index, backend)
        if not cap.isOpened():
            tried.append(str(backend))
            cap.release()
            continue
        if width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        return cap
    raise SystemExit(f"Failed to open camera index {camera_index}")


def _run_camera(args) -> int:
    ensure_runtime_dirs()
    preload_cuda_user_libs()

    detector = _build_face_detector()
    device = torch_device() if args.engine == "torch" else "cpu"

    from emotiefflib.facial_analysis import EmotiEffLibRecognizer

    recognizer = EmotiEffLibRecognizer(
        engine=args.engine,
        model_name=args.model,
        device=device,
    )

    cap = _open_camera(args.camera, args.camera_width, args.camera_height)
    window_name = "Momo Emotion Camera"
    frame_index = 0
    frame_timer = time.perf_counter()
    last_payload = {
        "camera": args.camera,
        "model": args.model,
        "engine": args.engine,
        "device": device,
        "label": "NoFace",
        "scores": {},
        "face_bbox": None,
        "frame_index": -1,
    }

    print("camera controls: press q or ESC to quit")
    try:
        while True:
            ok, frame_bgr = cap.read()
            if not ok or frame_bgr is None:
                raise SystemExit(f"Failed to read frame from camera {args.camera}")

            frame_index += 1
            if frame_index == 1 or frame_index % args.predict_every == 0:
                payload = _predict_emotion(frame_bgr, detector, recognizer, device)
                payload.update(
                    {
                        "camera": args.camera,
                        "model": args.model,
                        "engine": args.engine,
                        "frame_index": frame_index,
                        "frame_size": [int(frame_bgr.shape[1]), int(frame_bgr.shape[0])],
                        "timestamp": time.time(),
                    }
                )
                last_payload = payload
                write_json(args.output_json, payload)
                if args.save_frame is not None:
                    args.save_frame.parent.mkdir(parents=True, exist_ok=True)
                    cv2.imwrite(str(args.save_frame), frame_bgr)

            now = time.perf_counter()
            fps = 1.0 / max(now - frame_timer, 1e-6)
            frame_timer = now

            if args.display:
                annotated = _draw_overlay(frame_bgr, last_payload, fps)
                cv2.imshow(window_name, annotated)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break

            if args.max_frames is not None and frame_index >= args.max_frames:
                break
    finally:
        cap.release()
        if args.display:
            cv2.destroyAllWindows()

    print(f"camera: {args.camera}")
    print(f"label: {last_payload.get('label')}")
    print(f"json: {args.output_json}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Run local facial emotion recognition.")
    parser.add_argument("--image", type=Path, help="Path to an input image.")
    parser.add_argument("--demo-image", action="store_true", help="Use a downloaded demo face image.")
    parser.add_argument("--camera", type=int, help="Open a camera index for realtime emotion recognition.")
    parser.add_argument("--camera-width", type=int, help="Preferred capture width for camera mode.")
    parser.add_argument("--camera-height", type=int, help="Preferred capture height for camera mode.")
    parser.add_argument("--predict-every", type=int, default=3, help="Run emotion inference every N frames.")
    parser.add_argument("--max-frames", type=int, help="Stop automatically after N frames in camera mode.")
    parser.add_argument("--display", dest="display", action="store_true", default=True, help="Show preview window in camera mode.")
    parser.add_argument("--no-display", dest="display", action="store_false", help="Disable preview window in camera mode.")
    parser.add_argument("--save-frame", type=Path, help="Optional path to save the latest camera frame.")
    parser.add_argument("--engine", default=DEFAULT_EMOTION_ENGINE, choices=["torch", "onnx"])
    parser.add_argument("--model", default=DEFAULT_EMOTION_MODEL)
    parser.add_argument(
        "--output-json",
        type=Path,
        default=RUNS_ROOT / "emotion" / "latest.json",
        help="Where to save the prediction summary.",
    )
    args = parser.parse_args()

    mode_count = sum(
        1
        for enabled in (
            args.image is not None,
            args.demo_image,
            args.camera is not None,
        )
        if enabled
    )
    if mode_count != 1:
        raise SystemExit("Provide exactly one of --image, --demo-image, or --camera")

    if args.camera is not None:
        return _run_camera(args)

    ensure_runtime_dirs()
    preload_cuda_user_libs()
    if args.demo_image:
        image_path = save_demo_face()
    else:
        image_path = args.image

    image_bgr = cv2.imread(str(image_path))
    if image_bgr is None:
        raise SystemExit(f"Failed to load image: {image_path}")

    detector = _build_face_detector()
    device = torch_device() if args.engine == "torch" else "cpu"

    from emotiefflib.facial_analysis import EmotiEffLibRecognizer

    recognizer = EmotiEffLibRecognizer(
        engine=args.engine,
        model_name=args.model,
        device=device,
    )
    payload = _predict_emotion(image_bgr, detector, recognizer, device)
    payload = {
        "image": str(image_path),
        "model": args.model,
        "engine": args.engine,
        **payload,
    }
    write_json(args.output_json, payload)
    print(f"image: {image_path}")
    print(f"label: {payload['label']}")
    print(f"json: {args.output_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
