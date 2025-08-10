import json
import re
from typing import Any, Dict, List

# --- unit helpers ---


def _to_float(x):
    try:
        return float(x)
    except Exception:
        return None


def f_to_c(v):
    v = _to_float(v)
    return None if v is None else (v-32.0)*5.0/9.0


def kmh_to_mps(v):
    v = _to_float(v)
    return None if v is None else v/3.6


def mph_to_mps(v):
    v = _to_float(v)
    return None if v is None else v*0.44704


def pct_or_fraction_to_pct(v):
    v = _to_float(v)
    if v is None:
        return None
    return v*100.0 if abs(v) <= 1 else v


_COERCE = {
    "int": lambda x: int(_to_float(x)) if _to_float(x) is not None else None,
    "float": _to_float,
    None: lambda x: x,
}

_TRANSFORM = {
    "f_to_c": f_to_c,
    "kmh_to_mps": kmh_to_mps,
    "mph_to_mps": mph_to_mps,
    "pct_or_fraction_to_pct": pct_or_fraction_to_pct,
    None: lambda x: x,
}


def _clean_key(raw_key: str) -> str:
    """lowercase, strip namespaces, keep only the last segment after '.'"""
    if not isinstance(raw_key, str):
        return ""
    k = raw_key.strip().lower()
    segs = [s.split(":", 1)[-1] for s in k.split(".")]
    return segs[-1] if segs else k


def load_extension_rules(path: str) -> List[dict]:
    with open(path, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    rules = cfg.get("rules", [])
    # normalize: lowercase synonyms
    for r in rules:
        r["synonyms"] = [s.lower() for s in r.get("synonyms", [])]
        r["coerce"] = r.get("coerce")
        r["transform"] = r.get("transform")
    return rules


def normalize_extensions_kv(items: List[Dict[str, Any]],
                            rules: List[dict],
                            keep_unknown: bool = False) -> Dict[str, Any]:
    """
    items: [{"key":"...","value":...}, ...]  (keys already 'last segment' is fine)
    rules: from load_extension_rules()
    returns canonical dict like {"hr_bpm": 118, "power_w": 247, ...}
    """
    out: Dict[str, Any] = {}
    for it in items or []:
        ck = _clean_key(it.get("key"))
        if not ck:
            continue
        val = it.get("value")

        mapped = False
        for r in rules:                      # priority by order in JSON
            if ck in r["synonyms"]:
                v = _COERCE.get(r["coerce"], _COERCE[None])(val)
                v = _TRANSFORM.get(r["transform"], _TRANSFORM[None])(v)
                if v is not None and r["canonical"] not in out:
                    # first-wins; tweak if you want override logic
                    out[r["canonical"]] = v
                mapped = True
                break
        if not mapped and keep_unknown:
            out[f"ext_{ck}"] = val
    return out
