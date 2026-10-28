"""Entry point: ``python -m console.backend``."""

from __future__ import annotations

import uvicorn

from .app import app


def main() -> None:
    settings = app.state.settings
    uvicorn.run(
        app,
        host=settings.host,
        port=settings.port,
        log_level="info",
        access_log=False,
    )


if __name__ == "__main__":
    main()
