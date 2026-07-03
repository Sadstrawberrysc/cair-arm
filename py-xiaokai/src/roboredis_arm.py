import json
import os
import time
import uuid
from typing import Any, Dict, Optional, Tuple, Sequence

try:
    import redis
    from redis.exceptions import RedisError
except Exception:  # pragma: no cover - redis is optional at import time
    redis = None  # type: ignore
    RedisError = Exception  # type: ignore

DEFAULT_REDIS_URL = "redis://127.0.0.1:6379/0"
ARM_COMMAND_QUEUE = os.getenv("ARM_COMMAND_QUEUE", "device:arm:command")
ARM_STATUS_HASH = os.getenv("ARM_STATUS_HASH", "device:arm:status")
ARM_EVENT_CHANNEL = os.getenv("ARM_EVENT_CHANNEL", "device:arm:event")
ARM_DRAG_CHANNEL = os.getenv("ARM_DRAG_CHANNEL", "device:arm:drag")
ARM_MOVE_CHANNEL = os.getenv("ARM_MOVE_CHANNEL", "device:arm:move")

class RedisUnavailableError(RuntimeError):
    """Raised when the redis client cannot be created."""


def create_redis_client(redis_url: Optional[str] = None, *, decode_responses: bool = True):
    if redis is None:
        raise RedisUnavailableError(
            "redis package is not installed. Please install `redis` to use Redis-based arm control."
        )
    url = redis_url or os.getenv("REDIS_URL", DEFAULT_REDIS_URL)
    return redis.Redis.from_url(url, decode_responses=decode_responses, socket_keepalive=True)


class ArmCommandPublisher:
    """Publish structured arm commands to Redis."""

    def __init__(
        self,
        *,
        redis_url: Optional[str] = None,
        queue_name: Optional[str] = None,
        source: str = "unknown",
    ) -> None:
        self.source = source
        self.queue_name = queue_name or ARM_COMMAND_QUEUE
        self._client = create_redis_client(redis_url=redis_url, decode_responses=True)

    @property
    def client(self):
        return self._client

    def ping(self) -> Tuple[bool, str]:
        try:
            ok = bool(self._client.ping())
            return ok, "" if ok else "ping returned False"
        except RedisError as exc:  # pragma: no cover - network errors
            return False, str(exc)

    def _publish(self, command: Dict[str, Any]) -> bool:
        payload = {
            "id": command.get("id") or str(uuid.uuid4()),
            "source": command.get("source", self.source),
            "issued_at": command.get("issued_at", time.time()),
            **command,
        }
        try:
            serialized = json.dumps(payload, ensure_ascii=False)
            self._client.rpush(self.queue_name, serialized)
            return True
        except RedisError as exc:
            print(f"[arm|redis] publish failed: {exc}")
            return False

    def send_direction(self, direction: str, *, step_mm: Optional[float] = None, meta: Optional[Dict[str, Any]] = None) -> bool:
        command = {
            "command": "move_direction",
            "payload": {
                "direction": direction,
                "step_mm": step_mm,
                **(meta or {}),
            },
        }
        return self._publish(command)
    
    def send_dragrobot(self, flag: str)-> bool:
        command = {
            "command": "drag",
            "payload": {
                "drag_flag": flag
            },
        }
        return self._publish(command)
    
    def publish_drag(self, flag: str) -> bool:
        """通过 Pub/Sub 发布拖拽消息，不走队列，避免被 BLPOP 的 worker 消费。"""
        envelope = {
            "id": str(uuid.uuid4()),
            "source": self.source,
            "issued_at": time.time(),
            "command": "drag",
            "payload": {"drag_flag": flag},
        }
        try:
            serialized = json.dumps(envelope, ensure_ascii=False)
            # 发布到单独频道
            self._client.publish(ARM_DRAG_CHANNEL, serialized)
            return True
        except RedisError as exc:
            print(f"[arm|redis] publish_drag failed: {exc}")
            return False
        
    def publish_move(self, flag: str) -> bool:
        """通过 Pub/Sub 发布移动标志，不走队列，避免被 BLPOP 的 worker 消费。"""
        envelope = {
            "id": str(uuid.uuid4()),
            "source": self.source,
            "issued_at": time.time(),
            "command": "move",
            "payload": {"move_flag": flag},
        }
        try:
            serialized = json.dumps(envelope, ensure_ascii=False)
            # 发布到单独频道
            self._client.publish(ARM_MOVE_CHANNEL, serialized)
            return True
        except RedisError as exc:
            print(f"[arm|redis] publish_move failed: {exc}")
            return False

    def publish_joint_angles(
        self,
        angles: Sequence[float]
    ) -> bool:
        """
        通过 Pub/Sub 向 ARM_MOVE_CHANNEL 发布目标 6 关节角度。
        - angles: 长度为 6 的序列，按关节 J1..J6 顺序给出角度
        - unit: "deg" 或 "rad"
        返回 True 表示 publish 成功（仅代表消息已发出，不代表执行成功）
        """
        # 基本校验
        if not isinstance(angles, (list, tuple)) or len(angles) != 6:
            print("[arm|redis] publish_joint_angles failed: angles must be length-6 sequence")
            return False
        try:
            angles_list = [float(x) for x in angles]
        except Exception:
            print("[arm|redis] publish_joint_angles failed: angles must be numeric")
            return False

        envelope = {
            "id": str(uuid.uuid4()),
            "source": self.source,
            "issued_at": time.time(),
            "command": "move_joint_angles",
            "payload": {
                "angles": angles_list
            },
        }
        try:
            serialized = json.dumps(envelope, ensure_ascii=False)
            self._client.publish(ARM_MOVE_CHANNEL, serialized)
            return True
        except RedisError as exc:
            print(f"[arm|redis] publish_joint_angles failed: {exc}")
            return False
        
    def send_relative(self, *, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, meta: Optional[Dict[str, Any]] = None) -> bool:
        command = {
            "command": "move_relative",
            "payload": {
                "dx": dx,
                "dy": dy,
                "dz": dz,
                **(meta or {}),
            },
        }
        return self._publish(command)

    def send_custom(self, command: str, payload: Optional[Dict[str, Any]] = None) -> bool:
        envelope = {
            "command": command,
            "payload": payload or {},
        }
        return self._publish(envelope)

    def close(self) -> None:
        try:
            self._client.close()
        except Exception:
            pass


def update_status(client, *, status: Dict[str, Any], key: Optional[str] = None) -> None:
    try:
        target = key or ARM_STATUS_HASH
        client.hset(target, mapping=status)
    except RedisError as exc:  # pragma: no cover - best effort
        print(f"[arm|redis] failed to update status: {exc}")