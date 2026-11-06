"""Write the three GUI scripts after a successful, scored startup attempt."""
import argparse
import json
from pathlib import Path


def scripts():
    startup = json.loads((Path(__file__).parent / "stand_attempt.json").read_text())["events"][:-1]
    operations = {
        "S1": [(6., "press", "W"), (8., "release", "W"),
               (11., "press", "S"), (13., "release", "S")],
        "S2": [(6., "press", "Shift"), (9., "release", "Shift"),
               (11., "press", "W"), (12., "press", "Shift"),
               (14., "release", "W"), (16., "release", "Shift")],
        "S3": [(6., "press", "C"), (6.1, "release", "C"),
               (10., "press", "W"), (12., "release", "W"),
               (15., "press", "C"), (15.1, "release", "C")],
    }
    for name, operations_for_name in operations.items():
        events = list(startup)
        events.extend({"at": at, "op": operation, "key": key} for at, operation, key in operations_for_name)
        events.append({"at": 20., "op": "switch", "side": "right", "value": 99})
        yield name, {"description": "Real virtualDbus events; evaluate the actual Manual commands in received_topics.jsonl",
                     "duration": 22., "events": events}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for name, script in scripts():
        (args.output / (name + ".json")).write_text(json.dumps(script, indent=2) + "\n")
