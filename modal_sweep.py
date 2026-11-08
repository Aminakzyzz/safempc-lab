"""Modal-ready independent parameter sweep. Run only after `modal setup`."""

import itertools

try:
    import modal
except ImportError as exc:
    raise SystemExit("Install the optional dependency first: pip install '.[modal]'") from exc

app = modal.App("safempc-lab-sweep")
image = modal.Image.debian_slim(python_version="3.11").pip_install("numpy>=1.24")


@app.function(image=image)
def evaluate(speed: float, q_position: float) -> dict:
    """Independent trial hook for the MPC parameter sweep."""
    return {"speed": speed, "q_position": q_position, "status": "scaffold-ready"}


@app.local_entrypoint()
def main() -> None:
    trials = list(itertools.product((0.3, 0.45, 0.6), (4.0, 8.0, 12.0)))
    for result in evaluate.starmap(trials):
        print(result)
