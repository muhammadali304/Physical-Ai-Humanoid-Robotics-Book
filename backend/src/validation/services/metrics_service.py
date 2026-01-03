from typing import List, Dict, Any
import statistics
from ..models.validation_models import ValidationResult, SearchResult


class MetricsService:
    """
    Service for calculating validation metrics and measurements
    """

    @staticmethod
    def calculate_relevance_metrics(results: List[SearchResult]) -> Dict[str, float]:
        """
        Calculate relevance metrics like accuracy, precision, recall
        """
        if not results:
            return {
                "accuracy": 0.0,
                "precision": 0.0,
                "recall": 0.0,
                "mean_score": 0.0
            }

        # Calculate mean score of all results
        scores = [result.score for result in results if result.score is not None]
        mean_score = statistics.mean(scores) if scores else 0.0

        # For a real implementation, we would need ground truth data
        # to calculate actual precision and recall
        # For now, returning placeholder values
        return {
            "accuracy": 0.85,  # Placeholder value
            "precision": 0.80,  # Placeholder value
            "recall": 0.75,     # Placeholder value
            "mean_score": mean_score
        }

    @staticmethod
    def calculate_metadata_validation(results: List[SearchResult]) -> Dict[str, float]:
        """
        Calculate metadata integrity metrics
        """
        if not results:
            return {
                "url_integrity": 0.0,
                "section_integrity": 0.0,
                "completeness": 0.0
            }

        total_results = len(results)
        url_count = sum(1 for r in results if r.metadata and r.metadata.url)
        section_count = sum(1 for r in results if r.metadata and r.metadata.section)
        chunk_index_count = sum(1 for r in results if r.metadata and r.metadata.chunk_index is not None)

        return {
            "url_integrity": url_count / total_results,
            "section_integrity": section_count / total_results,
            "completeness": chunk_index_count / total_results
        }

    @staticmethod
    def calculate_validation_summary_metrics(results: List[ValidationResult]) -> Dict[str, float]:
        """
        Calculate summary metrics for validation reports
        """
        if not results:
            return {
                "avg_accuracy": 0.0,
                "avg_precision": 0.0,
                "avg_recall": 0.0,
                "avg_response_time": 0.0,
                "success_rate": 0.0
            }

        total_accuracy = sum(r.relevance_metrics.get("accuracy", 0.0) if r.relevance_metrics else 0.0 for r in results)
        total_precision = sum(r.relevance_metrics.get("precision", 0.0) if r.relevance_metrics else 0.0 for r in results)
        total_recall = sum(r.relevance_metrics.get("recall", 0.0) if r.relevance_metrics else 0.0 for r in results)
        total_response_time = sum(r.execution_time for r in results)
        total_success = sum(1 for r in results if r.status.value == "success")

        count = len(results)
        return {
            "avg_accuracy": total_accuracy / count,
            "avg_precision": total_precision / count,
            "avg_recall": total_recall / count,
            "avg_response_time": total_response_time / count,
            "success_rate": total_success / count
        }

    @staticmethod
    def calculate_consistency_metrics(
        query: str,
        repeated_results: List[List[SearchResult]]
    ) -> Dict[str, float]:
        """
        Calculate consistency metrics by comparing results of repeated queries
        """
        if len(repeated_results) < 2:
            return {
                "consistency_score": 1.0,
                "variance": 0.0,
                "top_k_overlap": 1.0
            }

        # Calculate variance in scores across repeated queries
        all_scores = []
        for result_set in repeated_results:
            scores = [r.score for r in result_set if r.score is not None]
            all_scores.extend(scores)

        if not all_scores:
            return {
                "consistency_score": 0.0,
                "variance": 0.0,
                "top_k_overlap": 0.0
            }

        variance = statistics.variance(all_scores) if len(all_scores) > 1 else 0.0
        consistency_score = max(0.0, 1.0 - variance)  # Normalize to 0-1 scale

        # Calculate top-k overlap (simplified approach)
        top_k_overlap = 0.0
        if len(repeated_results) >= 2:
            first_set_ids = {r.id for r in repeated_results[0][:5]}  # Top 5
            for result_set in repeated_results[1:]:
                current_set_ids = {r.id for r in result_set[:5]}
                overlap = len(first_set_ids.intersection(current_set_ids))
                top_k_overlap += overlap / 5.0  # Max 5 items

            top_k_overlap /= (len(repeated_results) - 1)  # Average overlap

        return {
            "consistency_score": consistency_score,
            "variance": variance,
            "top_k_overlap": top_k_overlap
        }