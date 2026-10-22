import json, statistics, sys
rows = [json.loads(l) for l in open(sys.argv[1], encoding="utf-8")]
d_cap = [r["anchor"] - r["capture"] for r in rows]
d_infer = [r["infer_done"] - r["anchor"] for r in rows]
s = sorted(d_cap)
print("anchor - capture (should be ~0): median %+.1f ms  p10 %+.1f  p90 %+.1f"
      % (statistics.median(d_cap) * 1000, s[len(s) // 10] * 1000, s[len(s) * 9 // 10] * 1000))
print("infer_done - anchor (should be ~infer time): median %+.1f ms"
      % (statistics.median(d_infer) * 1000))
leads = sorted(r["lead"] for r in rows)
print("lead: median %.0f ms  p10 %.0f  p90 %.0f"
      % (statistics.median(leads) * 1000, leads[len(leads) // 10] * 1000, leads[len(leads) * 9 // 10] * 1000))
aims = [r["aim_yaw"] for r in rows]
print("aim_yaw range: %.3f .. %.3f rad" % (min(aims), max(aims)))
