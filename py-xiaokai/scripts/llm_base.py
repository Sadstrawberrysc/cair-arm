import json
import re
import redis
from abc import ABC, abstractmethod
from src.utils.logging_config import get_logger

logger = get_logger(__name__)
TAG = "LLM"


class LLMProviderBase(ABC):
    @abstractmethod
    def response(self, session_id, dialogue):
        """LLM response generator"""
        pass

    def response_no_stream(self, system_prompt, user_prompt, **kwargs):
        try:
            # 构造对话格式
            dialogue = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt},
            ]
            result = ""
            for part in self.response("", dialogue, **kwargs):
                result += part
            logger.bind(tag=TAG).info(f"Ollama response generated: {result}")

            # 解析\json内容并通过redis发送
            self._parse_and_send_json(result)

            # 移除\json部分，返回纯文本
            cleaned_result = self._remove_json_from_result(result)
            return cleaned_result

        except Exception as e:
            logger.bind(tag=TAG).error(f"Error in Ollama response generation: {e}")
            return "【LLM服务响应异常】"

    def _parse_and_send_json(self, result: str):
        """从result中解析\json内容并通过redis发送"""
        try:
            # 使用正则表达式匹配 \json{...} 格式
            json_pattern = r"\\json\{(.+?)\}"
            matches = re.findall(json_pattern, result, re.DOTALL)

            if matches:
                for match in matches:
                    try:
                        # 解析JSON字符串
                        json_data = json.loads("{" + match + "}")

                        # 连接Redis并发送数据
                        redis_client = redis.Redis(
                            host="localhost", port=6379, db=0, decode_responses=True
                        )

                        # 发送到Redis队列，使用'xiaozhi:commands'作为队列名
                        redis_client.lpush(
                            "xiaozhi:commands",
                            json.dumps(json_data, ensure_ascii=False),
                        )

                        logger.bind(tag=TAG).info(
                            f"JSON command sent to Redis: {json_data}"
                        )

                    except json.JSONDecodeError as e:
                        logger.bind(tag=TAG).error(
                            f"Failed to parse JSON: {match}, error: {e}"
                        )
                    except redis.RedisError as e:
                        logger.bind(tag=TAG).error(f"Redis connection error: {e}")

        except Exception as e:
            logger.bind(tag=TAG).error(f"Error in JSON parsing and Redis sending: {e}")

    def _remove_json_from_result(self, result: str) -> str:
        """从result中移除\json部分，返回纯文本"""
        try:
            # 移除所有的 \json{...} 部分
            json_pattern = r"\\json\{[^}]*\}"
            cleaned_result = re.sub(json_pattern, "", result, flags=re.DOTALL)

            # 清理多余的空白字符
            cleaned_result = re.sub(r"\s+", " ", cleaned_result).strip()

            return cleaned_result

        except Exception as e:
            logger.bind(tag=TAG).error(f"Error removing JSON from result: {e}")
            return result

    def response_with_functions(self, session_id, dialogue, functions=None):
        """
        Default implementation for function calling (streaming)
        This should be overridden by providers that support function calls

        Returns: generator that yields either text tokens or a special function call token
        """
        # For providers that don't support functions, just return regular response
        for token in self.response(session_id, dialogue):
            yield token, None
