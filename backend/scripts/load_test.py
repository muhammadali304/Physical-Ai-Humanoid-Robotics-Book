"""
Load testing script for the RAG Ingestion Pipeline.
Tests system performance under various load conditions.
"""

import asyncio
import time
import json
import random
from typing import List, Dict, Any
from dataclasses import dataclass
from datetime import datetime

import httpx
from tqdm import tqdm


@dataclass
class LoadTestResult:
    """Result of a load test."""
    test_name: str
    start_time: datetime
    end_time: datetime
    total_requests: int
    successful_requests: int
    failed_requests: int
    total_duration: float
    avg_response_time: float
    min_response_time: float
    max_response_time: float
    error_rate: float
    throughput: float  # requests per second
    percentile_95: float
    percentile_99: float


class LoadTester:
    """Load testing class for the RAG Ingestion Pipeline."""

    def __init__(self, base_url: str, api_key: str, concurrency: int = 10):
        self.base_url = base_url
        self.api_key = api_key
        self.concurrency = concurrency
        self.headers = {
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json"
        }

    async def run_search_load_test(self, num_requests: int = 100) -> LoadTestResult:
        """
        Run a load test on the search endpoint.

        Args:
            num_requests: Number of requests to send

        Returns:
            LoadTestResult with test results
        """
        print(f"Starting search load test with {num_requests} requests at {self.concurrency} concurrency...")

        start_time = time.time()
        tasks = []

        # Create tasks for concurrent execution
        for i in range(num_requests):
            task = asyncio.create_task(self._single_search_request())
            tasks.append(task)

        # Execute all tasks with concurrency limit
        semaphore = asyncio.Semaphore(self.concurrency)

        async def limited_request(task):
            async with semaphore:
                return await task

        limited_tasks = [limited_request(task) for task in tasks]
        results = await asyncio.gather(*limited_tasks, return_exceptions=True)

        end_time = time.time()

        # Process results
        successful_requests = 0
        failed_requests = 0
        response_times = []

        for result in results:
            if isinstance(result, Exception):
                failed_requests += 1
                print(f"Request failed with exception: {result}")
            elif result is not None:
                successful_requests += 1
                response_times.append(result)
            else:
                failed_requests += 1

        # Calculate metrics
        total_duration = end_time - start_time
        total_requests = num_requests
        error_rate = failed_requests / total_requests if total_requests > 0 else 0
        throughput = total_requests / total_duration if total_duration > 0 else 0

        if response_times:
            avg_response_time = sum(response_times) / len(response_times)
            min_response_time = min(response_times)
            max_response_time = max(response_times)

            # Calculate percentiles
            sorted_times = sorted(response_times)
            percentile_95 = self._calculate_percentile(sorted_times, 95)
            percentile_99 = self._calculate_percentile(sorted_times, 99)
        else:
            avg_response_time = 0
            min_response_time = 0
            max_response_time = 0
            percentile_95 = 0
            percentile_99 = 0

        return LoadTestResult(
            test_name="Search Load Test",
            start_time=datetime.fromtimestamp(start_time),
            end_time=datetime.fromtimestamp(end_time),
            total_requests=total_requests,
            successful_requests=successful_requests,
            failed_requests=failed_requests,
            total_duration=total_duration,
            avg_response_time=avg_response_time,
            min_response_time=min_response_time,
            max_response_time=max_response_time,
            error_rate=error_rate,
            throughput=throughput,
            percentile_95=percentile_95,
            percentile_99=percentile_99
        )

    async def _single_search_request(self) -> float:
        """
        Make a single search request and return response time in seconds.

        Returns:
            Response time in seconds
        """
        start_time = time.time()

        try:
            async with httpx.AsyncClient(timeout=30.0) as client:
                response = await client.post(
                    f"{self.base_url}/search",
                    headers=self.headers,
                    json={
                        "query": "test search query for load testing",
                        "top_k": 5
                    }
                )

                response_time = time.time() - start_time

                if response.status_code != 200:
                    print(f"Search request failed with status {response.status_code}: {response.text}")
                    return None

                return response_time

        except Exception as e:
            print(f"Search request exception: {e}")
            return None

    async def run_crawl_load_test(self, num_requests: int = 10) -> LoadTestResult:
        """
        Run a load test on the crawl endpoint.

        Args:
            num_requests: Number of requests to send

        Returns:
            LoadTestResult with test results
        """
        print(f"Starting crawl load test with {num_requests} requests at {self.concurrency} concurrency...")

        start_time = time.time()
        tasks = []

        # Create tasks for concurrent execution
        for i in range(num_requests):
            task = asyncio.create_task(self._single_crawl_request())
            tasks.append(task)

        # Execute all tasks with concurrency limit
        semaphore = asyncio.Semaphore(self.concurrency)

        async def limited_request(task):
            async with semaphore:
                return await task

        limited_tasks = [limited_request(task) for task in tasks]
        results = await asyncio.gather(*limited_tasks, return_exceptions=True)

        end_time = time.time()

        # Process results
        successful_requests = 0
        failed_requests = 0
        response_times = []

        for result in results:
            if isinstance(result, Exception):
                failed_requests += 1
                print(f"Request failed with exception: {result}")
            elif result is not None:
                successful_requests += 1
                response_times.append(result)
            else:
                failed_requests += 1

        # Calculate metrics
        total_duration = end_time - start_time
        total_requests = num_requests
        error_rate = failed_requests / total_requests if total_requests > 0 else 0
        throughput = total_requests / total_duration if total_duration > 0 else 0

        if response_times:
            avg_response_time = sum(response_times) / len(response_times)
            min_response_time = min(response_times)
            max_response_time = max(response_times)

            # Calculate percentiles
            sorted_times = sorted(response_times)
            percentile_95 = self._calculate_percentile(sorted_times, 95)
            percentile_99 = self._calculate_percentile(sorted_times, 99)
        else:
            avg_response_time = 0
            min_response_time = 0
            max_response_time = 0
            percentile_95 = 0
            percentile_99 = 0

        return LoadTestResult(
            test_name="Crawl Load Test",
            start_time=datetime.fromtimestamp(start_time),
            end_time=datetime.fromtimestamp(end_time),
            total_requests=total_requests,
            successful_requests=successful_requests,
            failed_requests=failed_requests,
            total_duration=total_duration,
            avg_response_time=avg_response_time,
            min_response_time=min_response_time,
            max_response_time=max_response_time,
            error_rate=error_rate,
            throughput=throughput,
            percentile_95=percentile_95,
            percentile_99=percentile_99
        )

    async def _single_crawl_request(self) -> float:
        """
        Make a single crawl request and return response time in seconds.

        Returns:
            Response time in seconds
        """
        start_time = time.time()

        try:
            async with httpx.AsyncClient(timeout=30.0) as client:
                response = await client.post(
                    f"{self.base_url}/crawl",
                    headers=self.headers,
                    json={
                        "url": "https://example.com/docs",  # Using example URL for testing
                        "max_depth": 1,
                        "include_patterns": ["/docs/*"]
                    }
                )

                response_time = time.time() - start_time

                if response.status_code != 200:
                    print(f"Crawl request failed with status {response.status_code}: {response.text}")
                    return None

                return response_time

        except Exception as e:
            print(f"Crawl request exception: {e}")
            return None

    def _calculate_percentile(self, sorted_values: List[float], percentile: float) -> float:
        """
        Calculate percentile from sorted values.

        Args:
            sorted_values: Sorted list of values
            percentile: Percentile to calculate (e.g., 95 for 95th percentile)

        Returns:
            Calculated percentile value
        """
        if not sorted_values:
            return 0

        index = (percentile / 100.0) * (len(sorted_values) - 1)
        lower_index = int(index)
        upper_index = lower_index + 1

        if upper_index >= len(sorted_values):
            return sorted_values[-1]

        # Interpolate between values
        fraction = index - lower_index
        lower_value = sorted_values[lower_index]
        upper_value = sorted_values[upper_index]

        return lower_value + fraction * (upper_value - lower_value)

    async def run_health_check_load_test(self, num_requests: int = 1000) -> LoadTestResult:
        """
        Run a load test on the health check endpoint.

        Args:
            num_requests: Number of requests to send

        Returns:
            LoadTestResult with test results
        """
        print(f"Starting health check load test with {num_requests} requests at {self.concurrency} concurrency...")

        start_time = time.time()
        tasks = []

        # Create tasks for concurrent execution
        for i in range(num_requests):
            task = asyncio.create_task(self._single_health_check_request())
            tasks.append(task)

        # Execute all tasks with concurrency limit
        semaphore = asyncio.Semaphore(self.concurrency)

        async def limited_request(task):
            async with semaphore:
                return await task

        limited_tasks = [limited_request(task) for task in tasks]
        results = await asyncio.gather(*limited_tasks, return_exceptions=True)

        end_time = time.time()

        # Process results
        successful_requests = 0
        failed_requests = 0
        response_times = []

        for result in results:
            if isinstance(result, Exception):
                failed_requests += 1
                print(f"Request failed with exception: {result}")
            elif result is not None:
                successful_requests += 1
                response_times.append(result)
            else:
                failed_requests += 1

        # Calculate metrics
        total_duration = end_time - start_time
        total_requests = num_requests
        error_rate = failed_requests / total_requests if total_requests > 0 else 0
        throughput = total_requests / total_duration if total_duration > 0 else 0

        if response_times:
            avg_response_time = sum(response_times) / len(response_times)
            min_response_time = min(response_times)
            max_response_time = max(response_times)

            # Calculate percentiles
            sorted_times = sorted(response_times)
            percentile_95 = self._calculate_percentile(sorted_times, 95)
            percentile_99 = self._calculate_percentile(sorted_times, 99)
        else:
            avg_response_time = 0
            min_response_time = 0
            max_response_time = 0
            percentile_95 = 0
            percentile_99 = 0

        return LoadTestResult(
            test_name="Health Check Load Test",
            start_time=datetime.fromtimestamp(start_time),
            end_time=datetime.fromtimestamp(end_time),
            total_requests=total_requests,
            successful_requests=successful_requests,
            failed_requests=failed_requests,
            total_duration=total_duration,
            avg_response_time=avg_response_time,
            min_response_time=min_response_time,
            max_response_time=max_response_time,
            error_rate=error_rate,
            throughput=throughput,
            percentile_95=percentile_95,
            percentile_99=percentile_99
        )

    async def _single_health_check_request(self) -> float:
        """
        Make a single health check request and return response time in seconds.

        Returns:
            Response time in seconds
        """
        start_time = time.time()

        try:
            async with httpx.AsyncClient(timeout=30.0) as client:
                response = await client.get(
                    f"{self.base_url}/health"
                )

                response_time = time.time() - start_time

                if response.status_code != 200:
                    print(f"Health check request failed with status {response.status_code}: {response.text}")
                    return None

                return response_time

        except Exception as e:
            print(f"Health check request exception: {e}")
            return None

    def print_results(self, result: LoadTestResult):
        """
        Print load test results in a formatted way.

        Args:
            result: LoadTestResult to print
        """
        print(f"\n{'='*60}")
        print(f"LOAD TEST RESULTS: {result.test_name}")
        print(f"{'='*60}")
        print(f"Test Duration: {result.total_duration:.2f} seconds")
        print(f"Total Requests: {result.total_requests}")
        print(f"Successful Requests: {result.successful_requests}")
        print(f"Failed Requests: {result.failed_requests}")
        print(f"Error Rate: {result.error_rate:.2%}")
        print(f"Throughput: {result.throughput:.2f} requests/second")
        print(f"Average Response Time: {result.avg_response_time*1000:.2f} ms")
        print(f"Min Response Time: {result.min_response_time*1000:.2f} ms")
        print(f"Max Response Time: {result.max_response_time*1000:.2f} ms")
        print(f"95th Percentile: {result.percentile_95*1000:.2f} ms")
        print(f"99th Percentile: {result.percentile_99*1000:.2f} ms")
        print(f"{'='*60}")


async def main():
    """Main function to run load tests."""
    # Configuration - these should be set based on your environment
    BASE_URL = "http://localhost:8000"
    API_KEY = "your_api_key_here"  # Replace with your actual API key

    # Create load tester
    tester = LoadTester(BASE_URL, API_KEY, concurrency=20)

    # Run health check load test (lightweight test first)
    print("Running health check load test...")
    health_result = await tester.run_health_check_load_test(num_requests=500)
    tester.print_results(health_result)

    # Run search load test
    print("\nRunning search load test...")
    search_result = await tester.run_search_load_test(num_requests=100)
    tester.print_results(search_result)

    # Run crawl load test (with fewer requests since it's more intensive)
    print("\nRunning crawl load test...")
    crawl_result = await tester.run_crawl_load_test(num_requests=20)
    tester.print_results(crawl_result)

    # Summary
    print("\n" + "="*60)
    print("LOAD TEST SUMMARY")
    print("="*60)
    print(f"Health Check - Success Rate: {health_result.successful_requests/health_result.total_requests:.2%}, Avg Response: {health_result.avg_response_time*1000:.2f}ms")
    print(f"Search - Success Rate: {search_result.successful_requests/search_result.total_requests:.2%}, Avg Response: {search_result.avg_response_time*1000:.2f}ms")
    print(f"Crawl - Success Rate: {crawl_result.successful_requests/crawl_result.total_requests:.2%}, Avg Response: {crawl_result.avg_response_time*1000:.2f}ms")


if __name__ == "__main__":
    asyncio.run(main())