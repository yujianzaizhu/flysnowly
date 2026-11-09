"""两套仿真共用的唯一 HTTP 能力：查询和切换当前应用。"""


def handle(handler, method):
    server = handler.server
    controller = getattr(server, "mode_controller", None)
    if controller is None:
        return False
    if method == "GET" and handler.path == "/api/mode":
        handler._send_json(controller.snapshot())
        return True
    if method == "POST" and handler.path == "/api/sim_mode":
        try:
            payload = handler._read_json()
            result = controller.request_switch(server, str(payload.get("mode") or ""))
            handler._send_json(result)
        except ValueError as exc:
            handler._send_json({"error": str(exc)}, status=getattr(exc, "http_status", 400))
        return True
    return False
