import pytest
from unittest.mock import patch
from src.validation.config import settings


@pytest.fixture(autouse=True)
def mock_settings():
    """
    Mock the settings to avoid requiring real API keys for tests
    """
    with patch('src.validation.config.ValidationSettings') as mock_settings_class:
        mock_settings_instance = mock_settings_class.return_value
        mock_settings_instance.cohere_api_key = "test_cohere_key"
        mock_settings_instance.qdrant_url = "http://localhost:6333"
        mock_settings_instance.qdrant_api_key = "test_qdrant_key"
        mock_settings_instance.qdrant_collection_name = "test_collection"
        mock_settings_instance.validation_api_key = "test_api_key"
        mock_settings_instance.validation_rate_limit = "100/minute"
        mock_settings_instance.cohere_model = "embed-multilingual-v3.0"
        yield mock_settings_instance