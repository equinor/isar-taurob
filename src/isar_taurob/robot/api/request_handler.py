import logging
import warnings
from typing import Any

import requests
from requests.exceptions import HTTPError, RequestException
from requests.models import Response

from isar_taurob.config import settings


class RequestHandler:
    def __init__(self) -> None:
        self.logger = logging.getLogger(settings.LOGGER_NAME)

        self.auth: tuple = (settings.API_USERNAME, settings.API_PASSWORD)
        self.proxies: dict = {"https": settings.API_PROXY}

    def _base_request(
        self,
        url: str,
        method: str,
        json_body: Any,
        timeout: float,
        headers: dict | None = None,
        data: dict | None = None,
        params: dict | None = None,
        **kwargs,
    ) -> Response:
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                response = requests.request(
                    url=url,
                    method=method,
                    auth=self.auth,
                    headers=headers,
                    timeout=timeout,
                    json=json_body,
                    data=data,
                    params=params,
                    verify=False,
                    proxies=self.proxies,
                    **kwargs,
                )
        except RequestException:
            raise
        except Exception as e:
            self.logger.exception("An unhandled exception occurred during a request")
            # Raise RequestException instead to avoid crashing
            raise RequestException from e

        try:
            response.raise_for_status()
        except HTTPError as e:
            response_dict: dict = e.response.json()
            try:
                error_message = response_dict["title"]
                raise RequestException(error_message)
            except KeyError:
                self.logger.exception(
                    f"Http error. Http status code= {response.status_code}. Http response body= {response.json()}"
                )
                raise e
        return response

    def get(
        self,
        url: str,
        json_body=None,
        request_timeout: float = settings.ROBOT_REQUESTS_TIMEOUT,
        headers: dict | None = None,
        data: dict | None = None,
        params: dict | None = None,
        **kwargs,
    ) -> Response:
        return self._base_request(
            url=url,
            method="GET",
            headers=headers,
            timeout=request_timeout,
            json_body=json_body,
            data=data,
            params=params,
            **kwargs,
        )

    def post(
        self,
        url: str,
        json_body=None,
        request_timeout: float = settings.ROBOT_REQUESTS_TIMEOUT,
        headers: dict | None = None,
        data: dict | None = None,
        params: dict | None = None,
        **kwargs,
    ) -> Response:
        self.logger.info(f"Sending POST request to {url} with {json_body}")
        return self._base_request(
            url=url,
            method="POST",
            headers=headers,
            timeout=request_timeout,
            json_body=json_body,
            data=data,
            params=params,
            **kwargs,
        )

    def delete(
        self,
        url: str,
        json_body=None,
        request_timeout: float = settings.ROBOT_REQUESTS_TIMEOUT,
        headers: dict | None = None,
        data: dict | None = None,
        params: dict | None = None,
        **kwargs,
    ) -> Response:
        return self._base_request(
            url=url,
            method="DELETE",
            headers=headers,
            timeout=request_timeout,
            json_body=json_body,
            data=data,
            params=params,
            **kwargs,
        )

    def put(
        self,
        url: str,
        json_body=None,
        request_timeout: float = settings.ROBOT_REQUESTS_TIMEOUT,
        headers: dict | None = None,
        data: dict | None = None,
        params: dict | None = None,
        **kwargs,
    ) -> Response:
        return self._base_request(
            url=url,
            method="PUT",
            headers=headers,
            timeout=request_timeout,
            json_body=json_body,
            data=data,
            params=params,
            **kwargs,
        )
