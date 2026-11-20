"""Minimal shared-token authentication.

Read-only views (map, video, fleet state) are intentionally anonymous so a
judge can open the cpolar link and watch.  Every mutating endpoint requires the
operator token, sent as a cookie after login, an ``X-Console-Token`` header, or
a ``?token=`` query parameter.
"""

from __future__ import annotations

from fastapi import HTTPException, Request

COOKIE_NAME = "mine_console_token"


def token_from_request(request: Request) -> str:
    token = request.cookies.get(COOKIE_NAME, "")
    if token:
        return token
    header = request.headers.get("x-console-token", "")
    if header:
        return header
    return request.query_params.get("token", "")


def is_operator(request: Request) -> bool:
    settings = request.app.state.settings
    return bool(token_from_request(request)) and token_from_request(request) == settings.token


async def require_operator(request: Request) -> str:
    if not is_operator(request):
        raise HTTPException(status_code=401, detail="需要操作员登录")
    return "operator"
