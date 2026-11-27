import logging

from azure.storage.blob import BlobServiceClient, ContainerClient

from isar.config.settings import settings
from isar.storage.storage_interface import (
    StorageException,
)


class BlobStorage:
    def __init__(self) -> None:
        self.logger = logging.getLogger("taurob-blob-storage")

        self.container_client_data = self._get_container_client(
            settings.BLOB_STORAGE_ACCOUNT_DATA,
            settings.BLOB_STORAGE_CONNECTION_STRING_DATA,
        )

    def _get_container_client(
        self, account_name: str, storage_connection_string: str
    ) -> ContainerClient:

        if not storage_connection_string:
            raise RuntimeError("BlobStorage connection string is Empty or None")

        try:
            blob_service_client = BlobServiceClient.from_connection_string(
                storage_connection_string
            )
            if blob_service_client.account_name != account_name:
                raise StorageException(
                    f"The blob storage in the connection string does not match the settings for blob storage account {account_name}"
                )
        except (StorageException, ValueError) as e:
            self.logger.error("Unable to retrieve blob service client. Error: %s", e)
            raise

        container_client: ContainerClient = blob_service_client.get_container_client(
            settings.BLOB_CONTAINER
        )

        if not container_client.exists():
            raise RuntimeError(
                "The configured blob container %s does not exist",
                settings.BLOB_CONTAINER,
            )
        return container_client

    def download_cost_map(self, asset_name: str, map_name: str) -> bytes:
        blob_client = self.container_client_data.get_blob_client(
            f"{asset_name}/{map_name}"
        )
        try:
            download_stream = blob_client.download_blob()
            data = download_stream.readall()
        except Exception as e:
            self.logger.error(
                "An unexpected error occurred while downloading map %s for asset %s: %s",
                map_name,
                asset_name,
                e,
            )
            raise StorageException from e

        return data
