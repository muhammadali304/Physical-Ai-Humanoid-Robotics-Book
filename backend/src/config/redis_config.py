"""
Redis configuration for the RAG Ingestion Pipeline.
Implements Redis server configuration for message queuing with RQ.
"""

from typing import Optional
from pydantic import BaseModel, Field
from urllib.parse import urlparse


class RedisConfig(BaseModel):
    """
    Configuration model for Redis server connection.
    """
    host: str = Field(default="localhost", description="Redis server host")
    port: int = Field(default=6379, description="Redis server port")
    db: int = Field(default=0, description="Redis database number")
    password: Optional[str] = Field(default=None, description="Redis server password")
    username: Optional[str] = Field(default=None, description="Redis server username")
    ssl: bool = Field(default=False, description="Use SSL for Redis connection")
    ssl_cert_reqs: Optional[str] = Field(default="required", description="SSL certificate requirements")
    ssl_ca_certs: Optional[str] = Field(default=None, description="Path to CA certificates for SSL")
    ssl_certfile: Optional[str] = Field(default=None, description="Path to SSL certificate file")
    ssl_keyfile: Optional[str] = Field(default=None, description="Path to SSL key file")
    connection_timeout: int = Field(default=30, description="Connection timeout in seconds")
    retry_on_timeout: bool = Field(default=True, description="Retry on connection timeout")
    max_connections: int = Field(default=20, description="Maximum number of connections")
    health_check_interval: int = Field(default=30, description="Health check interval in seconds")

    class Config:
        env_prefix = "REDIS_"
        case_sensitive = False
        env_file = ".env"

    def get_redis_url(self) -> str:
        """
        Generate a Redis URL from the configuration.

        Returns:
            Redis URL string
        """
        scheme = "rediss" if self.ssl else "redis"
        auth = ""
        if self.username and self.password:
            auth = f"{self.username}:{self.password}@"
        elif self.password:
            auth = f":{self.password}@"

        url = f"{scheme}://{auth}{self.host}:{self.port}/{self.db}"
        return url

    def get_rq_connection_kwargs(self) -> dict:
        """
        Get connection kwargs for RQ (Redis Queue).

        Returns:
            Dictionary of connection parameters for RQ
        """
        kwargs = {
            "host": self.host,
            "port": self.port,
            "db": self.db,
            "socket_connect_timeout": self.connection_timeout,
            "socket_timeout": self.connection_timeout,
            "health_check_interval": self.health_check_interval,
            "retry_on_timeout": self.retry_on_timeout,
            "max_connections": self.max_connections
        }

        if self.password:
            kwargs["password"] = self.password
        if self.username:
            kwargs["username"] = self.username
        if self.ssl:
            kwargs["ssl"] = self.ssl
            kwargs["ssl_cert_reqs"] = self.ssl_cert_reqs
            if self.ssl_ca_certs:
                kwargs["ssl_ca_certs"] = self.ssl_ca_certs
            if self.ssl_certfile:
                kwargs["ssl_certfile"] = self.ssl_certfile
            if self.ssl_keyfile:
                kwargs["ssl_keyfile"] = self.ssl_keyfile

        return kwargs


def get_redis_config_from_env() -> RedisConfig:
    """
    Create a RedisConfig instance from environment variables.

    Returns:
        RedisConfig instance with values loaded from environment
    """
    import os
    from pydantic import ValidationError

    try:
        # Load config from environment variables
        config = RedisConfig()
        return config
    except ValidationError as e:
        raise ValueError(f"Invalid Redis configuration: {e}")


def create_default_redis_connection():
    """
    Create a default Redis connection based on configuration.

    Returns:
        Redis connection object
    """
    import redis

    config = get_redis_config_from_env()
    connection_kwargs = config.get_rq_connection_kwargs()

    # Create Redis connection
    redis_conn = redis.Redis(**connection_kwargs)

    return redis_conn


def create_rq_queue(name: str = "default", connection=None) -> 'Queue':
    """
    Create an RQ queue with the specified name.

    Args:
        name: Name of the queue
        connection: Redis connection object (if None, creates default connection)

    Returns:
        RQ Queue object
    """
    from rq import Queue

    if connection is None:
        connection = create_default_redis_connection()

    return Queue(name=name, connection=connection)


# Default configuration instance
redis_config = get_redis_config_from_env()