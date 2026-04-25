#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import os
import random
import re
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

from common import REPO_ROOT


INGREDIENT_POINTS = {
    "01": {"name": "朗姆酒", "english": "Rum"},
    "02": {"name": "菠萝汁", "english": "Pineapple"},
    "03": {"name": "厚椰乳", "english": "Coconut"},
    "04": {"name": "蓝橙力娇酒", "english": "Blue Curacao"},
    "05": {"name": "威士忌", "english": "Whisky"},
    "06": {"name": "橙汁", "english": "Orange"},
    "07": {"name": "暴打柠檬拉格啤酒", "english": "Lemon Lager"},
}

VALID_EMOTIONS = {
    "Anger",
    "Contempt",
    "Disgust",
    "Fear",
    "Happiness",
    "Neutral",
    "Sadness",
    "Surprise",
}

FALLBACK_RECIPES = {
    "Happiness": [
        {
            "drink_name": "椰风晴空",
            "english_name": "Sunny Coconut Pop",
            "reason": "开心的时候适合明亮、热带、带一点轻盈甜感的组合。",
            "sequence": ["04", "02", "03"],
            "amounts_ml": {"04": 20, "02": 45, "03": 35},
            "style_tags": ["明亮", "热带", "活泼"],
        },
        {
            "drink_name": "海岛莫莫",
            "english_name": "Momo Island",
            "reason": "用经典热带组合把开心情绪再往上推一点。",
            "sequence": ["01", "02", "03"],
            "amounts_ml": {"01": 40, "02": 50, "03": 30},
            "style_tags": ["热带", "轻甜", "愉快"],
        },
    ],
    "Neutral": [
        {
            "drink_name": "平衡高球",
            "english_name": "Balanced Highball",
            "reason": "情绪平稳时更适合经典、清晰、接受度高的风味。",
            "sequence": ["05", "06"],
            "amounts_ml": {"05": 35, "06": 65},
            "style_tags": ["经典", "平衡", "安全"],
        },
        {
            "drink_name": "蓝色微风",
            "english_name": "Blue Breeze",
            "reason": "轻一点的果香和椰感，适合放松又不过分刺激。",
            "sequence": ["04", "03", "02"],
            "amounts_ml": {"04": 18, "03": 32, "02": 50},
            "style_tags": ["柔和", "清爽", "平稳"],
        },
    ],
    "Sadness": [
        {
            "drink_name": "暖橙安慰",
            "english_name": "Soft Orange Comfort",
            "reason": "偏柔和的果香和一点厚度，比较适合低落时慢慢喝。",
            "sequence": ["05", "06", "03"],
            "amounts_ml": {"05": 25, "06": 50, "03": 25},
            "style_tags": ["柔和", "安抚", "顺口"],
        },
        {
            "drink_name": "椰乳落日",
            "english_name": "Coconut Sunset",
            "reason": "用椰乳把口感做得更圆润，降低刺激感。",
            "sequence": ["01", "03", "06"],
            "amounts_ml": {"01": 30, "03": 35, "06": 35},
            "style_tags": ["圆润", "温和", "放松"],
        },
    ],
    "Surprise": [
        {
            "drink_name": "神秘蓝点",
            "english_name": "Mystery Blue",
            "reason": "惊喜情绪适合颜色感明显、风味反差更鲜明的组合。",
            "sequence": ["04", "07", "06"],
            "amounts_ml": {"04": 15, "07": 55, "06": 30},
            "style_tags": ["跳跃", "有趣", "反差"],
        },
        {
            "drink_name": "闪光热带",
            "english_name": "Spark Tropic",
            "reason": "热带甜香加啤酒气泡，比较有戏剧性。",
            "sequence": ["07", "02", "03"],
            "amounts_ml": {"07": 50, "02": 30, "03": 20},
            "style_tags": ["气泡", "热带", "戏剧感"],
        },
    ],
    "Fear": [
        {
            "drink_name": "轻柔护栏",
            "english_name": "Soft Guardrail",
            "reason": "紧张时先用更顺口、更熟悉的组合降低刺激。",
            "sequence": ["06", "03", "02"],
            "amounts_ml": {"06": 40, "03": 30, "02": 30},
            "style_tags": ["低刺激", "柔和", "熟悉"],
        }
    ],
    "Anger": [
        {
            "drink_name": "冷静蓝调",
            "english_name": "Cool Blue Down",
            "reason": "需要一点冷色系和清爽感，把情绪拉回平衡。",
            "sequence": ["04", "06", "02"],
            "amounts_ml": {"04": 15, "06": 45, "02": 40},
            "style_tags": ["降温", "清爽", "平衡"],
        }
    ],
    "Contempt": [
        {
            "drink_name": "克制高球",
            "english_name": "Dry Distance",
            "reason": "保持利落、干净，不做过度堆叠。",
            "sequence": ["05", "07"],
            "amounts_ml": {"05": 30, "07": 70},
            "style_tags": ["克制", "干净", "利落"],
        }
    ],
    "Disgust": [
        {
            "drink_name": "清口模式",
            "english_name": "Reset Sip",
            "reason": "优先清爽和简单，让味觉压力尽量小一点。",
            "sequence": ["06", "07"],
            "amounts_ml": {"06": 30, "07": 70},
            "style_tags": ["清爽", "简单", "重置"],
        }
    ],
}


def _load_dotenv(path: Path | None = None) -> dict[str, str]:
    env_path = path or (REPO_ROOT / ".env")
    values: dict[str, str] = {}
    if not env_path.exists():
        return values

    for raw_line in env_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        value = value.strip().strip('"').strip("'")
        values[key.strip()] = value
    return values


def _env_value(name: str, default: str | None = None) -> str | None:
    dotenv = _load_dotenv()
    return os.getenv(name) or dotenv.get(name) or default


def _emotion_path() -> Path:
    configured = _env_value("MOMOTENDER_EMOTION_JSON", "Software/ai/runs/emotion/latest.json")
    path = Path(configured)
    return path if path.is_absolute() else (REPO_ROOT / path)


def _read_emotion_state() -> dict[str, Any]:
    path = _emotion_path()
    if not path.exists():
        return {
            "label": "Neutral",
            "confidence": 0.5,
            "has_face": False,
            "scores": {},
            "path": str(path),
        }

    payload = json.loads(path.read_text(encoding="utf-8"))
    label = payload.get("label") or "Neutral"
    scores = payload.get("scores") or {}
    confidence = float(scores.get(label, 0.0))
    has_face = payload.get("face_bbox") is not None
    if label not in VALID_EMOTIONS:
        label = "Neutral"

    return {
        "label": label,
        "confidence": confidence,
        "has_face": has_face,
        "scores": scores,
        "path": str(path),
    }


def _normalized_emotion(emotion: dict[str, Any]) -> str:
    if not emotion["has_face"] or emotion["confidence"] < 0.45:
        return "Neutral"
    return emotion["label"]


def _recipe_lines(sequence: list[str], amounts_ml: dict[str, int]) -> list[str]:
    return [f"{INGREDIENT_POINTS[point]['name']} {int(amounts_ml[point])}ml" for point in sequence]


def _profile(sequence: list[str]) -> str:
    return " · ".join(INGREDIENT_POINTS[point]["english"] for point in sequence)


def _build_prompt(emotion: dict[str, Any]) -> tuple[str, str]:
    inventory = "\n".join(
        f"- {point}: {info['name']} / {info['english']}"
        for point, info in INGREDIENT_POINTS.items()
    )
    instructions = (
        "You are MomoTender, a playful cocktail recommendation assistant for an emotion-aware robot bartender. "
        "You must only use the allowed ingredient points and return compact JSON only. "
        "Do not use markdown, code fences, or extra commentary. "
        "The JSON keys must be exactly: drink_name, english_name, reason, sequence, amounts_ml, style_tags. "
        "sequence must contain 2 or 3 unique point ids chosen only from 01 to 07. "
        "amounts_ml must be an object whose keys match the chosen point ids and whose values are integers between 15 and 70. "
        "reason must be one short Chinese sentence. "
        "style_tags must contain 2 or 3 short Chinese tags."
    )
    user_input = (
        f"Detected emotion label: {emotion['label']}\n"
        f"Detected confidence: {emotion['confidence']:.2f}\n"
        f"Has face: {emotion['has_face']}\n"
        "Available ingredient points:\n"
        f"{inventory}\n"
        "Return one fun recommendation for a self-service robot bartender that can physically use only those fixed points."
    )
    return instructions, user_input


def _extract_json(text: str) -> dict[str, Any]:
    candidate = text.strip()
    if candidate.startswith("```"):
        candidate = re.sub(r"^```[a-zA-Z0-9_-]*\n?", "", candidate)
        candidate = re.sub(r"\n?```$", "", candidate)
    try:
        return json.loads(candidate)
    except json.JSONDecodeError:
        match = re.search(r"\{.*\}", candidate, re.S)
        if not match:
            raise
        return json.loads(match.group(0))


def _validate_llm_payload(payload: dict[str, Any]) -> dict[str, Any]:
    sequence = payload.get("sequence")
    if not isinstance(sequence, list) or not 2 <= len(sequence) <= 3:
        raise ValueError("sequence must contain 2 or 3 point ids")
    if len(set(sequence)) != len(sequence):
        raise ValueError("sequence must use unique points")
    if any(point not in INGREDIENT_POINTS for point in sequence):
        raise ValueError("sequence contains unavailable point")

    amounts_ml = payload.get("amounts_ml")
    if not isinstance(amounts_ml, dict):
        raise ValueError("amounts_ml must be an object")
    normalized_amounts: dict[str, int] = {}
    for point in sequence:
        if point not in amounts_ml:
            raise ValueError(f"missing amount for point {point}")
        amount = int(amounts_ml[point])
        if amount < 15 or amount > 70:
            raise ValueError(f"amount for point {point} out of range")
        normalized_amounts[point] = amount

    style_tags = payload.get("style_tags")
    if not isinstance(style_tags, list) or not 1 <= len(style_tags) <= 3:
        raise ValueError("style_tags must contain 1 to 3 items")

    reason = str(payload.get("reason", "")).strip()
    drink_name = str(payload.get("drink_name", "")).strip()
    english_name = str(payload.get("english_name", "")).strip()
    if not drink_name or not reason:
        raise ValueError("drink_name and reason are required")

    return {
        "drink_name": drink_name,
        "english_name": english_name or "MomoTender Special",
        "reason": reason,
        "sequence": sequence,
        "amounts_ml": normalized_amounts,
        "style_tags": [str(tag).strip() for tag in style_tags if str(tag).strip()],
    }


def _call_llm(emotion: dict[str, Any]) -> dict[str, Any]:
    base_url = _env_value("MOMOTENDER_API_BASE_URL")
    api_key = _env_value("MOMOTENDER_API_KEY")
    model = _env_value("MOMOTENDER_API_MODEL", "gpt-4.1-mini")
    timeout_seconds = float(_env_value("MOMOTENDER_API_TIMEOUT_SECONDS", "5.0"))
    max_output_tokens = int(_env_value("MOMOTENDER_API_MAX_OUTPUT_TOKENS", "220"))
    temperature = float(_env_value("MOMOTENDER_API_TEMPERATURE", "1.15"))

    if not base_url or not api_key:
        raise RuntimeError("MomoTender API environment is incomplete")

    instructions, user_input = _build_prompt(emotion)
    body: dict[str, Any] = {
        "model": model,
        "instructions": instructions,
        "input": user_input,
        "max_output_tokens": max_output_tokens,
        "temperature": temperature,
        "store": False,
    }
    if model.startswith("gpt-5"):
        body["reasoning"] = {"effort": "low"}

    request = urllib.request.Request(
        f"{base_url.rstrip('/')}/responses",
        data=json.dumps(body).encode("utf-8"),
        headers={
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
        },
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=timeout_seconds) as response:
        payload = json.loads(response.read().decode("utf-8"))
    if payload.get("error"):
        message = payload["error"].get("message") or str(payload["error"])
        raise RuntimeError(f"MomoTender API error: {message}")

    text_parts = []
    for item in payload.get("output", []):
        for content in item.get("content", []):
            if content.get("type") == "output_text":
                text_parts.append(content.get("text", ""))
    if not text_parts:
        raise RuntimeError("MomoTender API returned no output_text payload")

    parsed = _extract_json("\n".join(text_parts))
    validated = _validate_llm_payload(parsed)
    validated["model"] = model
    return validated


def _fallback_recommendation(emotion: dict[str, Any], error: str | None = None) -> dict[str, Any]:
    normalized = _normalized_emotion(emotion)
    choices = FALLBACK_RECIPES.get(normalized) or FALLBACK_RECIPES["Neutral"]
    selected = random.choice(choices)
    return {
        **selected,
        "source": "fallback",
        "model": None,
        "fallback_reason": error,
    }


def _shape_recommendation(raw: dict[str, Any], emotion: dict[str, Any], source: str) -> dict[str, Any]:
    sequence = raw["sequence"]
    amounts_ml = raw["amounts_ml"]
    return {
        "id": 5,
        "name": raw["drink_name"],
        "english": raw["english_name"],
        "profile": _profile(sequence),
        "reason": raw["reason"],
        "recipe": _recipe_lines(sequence, amounts_ml),
        "sequence": sequence,
        "amountsMl": amounts_ml,
        "auxiliary": [
            f"情绪 {emotion['label']} {emotion['confidence']:.2f}",
            f"来源 {'LLM' if source == 'llm' else 'Fallback'}",
        ],
        "styleTags": raw.get("style_tags", []),
        "image": "pic/mysterious.png",
        "accent": "#bba36a",
        "featureScale": 1.05,
        "menuScale": 1.02,
    }


def recommend_for_web() -> dict[str, Any]:
    emotion = _read_emotion_state()
    try:
        llm_payload = _call_llm(emotion)
        recommendation = _shape_recommendation(llm_payload, emotion, "llm")
        return {
            "ok": True,
            "source": "llm",
            "emotion": emotion,
            "recommendation": recommendation,
            "model": llm_payload["model"],
        }
    except (RuntimeError, ValueError, json.JSONDecodeError, urllib.error.URLError, TimeoutError, OSError) as exc:
        fallback = _fallback_recommendation(emotion, str(exc))
        recommendation = _shape_recommendation(fallback, emotion, "fallback")
        return {
            "ok": True,
            "source": "fallback",
            "emotion": emotion,
            "recommendation": recommendation,
            "model": None,
            "error": str(exc),
        }


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate a MomoTender recommendation.")
    parser.add_argument("--pretty", action="store_true")
    args = parser.parse_args()
    payload = recommend_for_web()
    if args.pretty:
        print(json.dumps(payload, ensure_ascii=False, indent=2))
    else:
        print(json.dumps(payload, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
