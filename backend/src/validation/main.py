from fastapi import FastAPI
from .api.routes import validation, health
from .config import settings
import logging


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


def create_app():
    """
    Create and configure the FastAPI application
    """
    app = FastAPI(
        title="RAG Validation API",
        description="API for validating RAG pipeline retrieval functionality",
        version="1.0.0"
    )

    # Include API routes
    app.include_router(validation.router)
    app.include_router(health.router)

    @app.on_event("startup")
    async def startup_event():
        """
        Startup event handler
        """
        logger.info("Starting up RAG Validation API...")
        # Add any startup logic here

    @app.on_event("shutdown")
    async def shutdown_event():
        """
        Shutdown event handler
        """
        logger.info("Shutting down RAG Validation API...")
        # Add any cleanup logic here

    return app


# Create the application instance
app = create_app()


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True
    )