#!/usr/bin/env python3
# scripts/validation/camera/front_camera_validation/image_quality_analyzer.py
# Generated: 2025-06-27
# Purpose: Unitree Go2 front camera image quality analysis

import cv2
import numpy as np
import logging
import statistics
from typing import Dict, List, Tuple, Any, Optional
from dataclasses import dataclass, field
from skimage import measure, filters, exposure
from skimage.metrics import structural_similarity as ssim
import matplotlib.pyplot as plt
from camera_config import CameraConfig

@dataclass
class ImageQualityMetrics:
    """Image quality metrics data class"""
    resolution_actual: Tuple[int, int] = (0, 0)
    resolution_target: Tuple[int, int] = (1280, 720)
    resolution_match: bool = False

    # Color metrics
    color_accuracy_score: float = 0.0
    color_delta_e: float = 0.0
    white_balance_score: float = 0.0
    saturation_score: float = 0.0

    # Sharpness metrics
    sharpness_score: float = 0.0
    blur_detection_score: float = 0.0
    edge_density: float = 0.0

    # Exposure and contrast
    brightness_score: float = 0.0
    contrast_score: float = 0.0
    dynamic_range: float = 0.0
    exposure_accuracy: float = 0.0

    # Noise metrics
    noise_level: float = 0.0
    snr_db: float = 0.0

    # Overall quality
    overall_quality_score: float = 0.0
    quality_grade: str = "UNKNOWN"

    # Raw data
    sample_images: List[np.ndarray] = field(default_factory=list)
    quality_history: List[float] = field(default_factory=list)

class ImageQualityAnalyzer:
    """Image quality analyzer"""

    def __init__(self, camera_config: CameraConfig, config: Dict[str, Any] = None):
        """
        Initialize image quality analyzer

        Args:
            camera_config: Camera configuration object
            config: Analysis configuration
        """
        self.camera_config = camera_config
        self.config = config or {}
        self.logger = logging.getLogger(__name__)

        # Reference images and templates
        self.reference_patterns = self._generate_reference_patterns()
        self.color_checker = self._create_color_checker()

    def analyze_image_quality(self, sample_count: int = 20) -> ImageQualityMetrics:
        """
        Comprehensive image quality analysis

        Args:
            sample_count: Number of sample images to analyze

        Returns:
            ImageQualityMetrics: Image quality analysis results
        """
        self.logger.info(f"Starting image quality analysis, sample count: {sample_count}")

        metrics = ImageQualityMetrics()

        # Collect sample images
        sample_images = self._collect_sample_images(sample_count)
        if not sample_images:
            self.logger.error("Unable to collect valid sample images")
            return metrics

        metrics.sample_images = sample_images[:5]  # Keep only first 5 as examples

        # Resolution verification
        metrics = self._analyze_resolution(sample_images, metrics)

        # Color analysis
        metrics = self._analyze_color_quality(sample_images, metrics)

        # Sharpness analysis
        metrics = self._analyze_sharpness(sample_images, metrics)

        # Exposure and contrast analysis
        metrics = self._analyze_exposure_contrast(sample_images, metrics)

        # Noise analysis
        metrics = self._analyze_noise(sample_images, metrics)

        # Calculate overall quality score
        metrics = self._calculate_overall_quality(metrics)

        self.logger.info(f"Image quality analysis complete, overall score: {metrics.overall_quality_score:.2f}")

        return metrics

    def _collect_sample_images(self, count: int) -> List[np.ndarray]:
        """Collect sample images"""
        images = []

        for i in range(count * 2):  # Collect extra in case some fail
            ret, frame = self.camera_config.capture_frame()
            if ret and frame is not None:
                images.append(frame.copy())
                if len(images) >= count:
                    break

            # Appropriate interval to get different images
            import time
            time.sleep(0.1)

        self.logger.info(f"Successfully collected {len(images)} sample images")
        return images

    def _analyze_resolution(self, images: List[np.ndarray],
                          metrics: ImageQualityMetrics) -> ImageQualityMetrics:
        """Analyze resolution"""
        if not images:
            return metrics

        # Get actual resolution
        height, width = images[0].shape[:2]
        metrics.resolution_actual = (width, height)

        # Check if it matches target resolution
        target_resolution = self.config.get("camera_config", {}).get("target_resolution", [1280, 720])
        metrics.resolution_target = tuple(target_resolution)

        metrics.resolution_match = (
            metrics.resolution_actual[0] == metrics.resolution_target[0] and
            metrics.resolution_actual[1] == metrics.resolution_target[1]
        )

        self.logger.info(f"Resolution analysis: actual {metrics.resolution_actual}, "
                        f"target {metrics.resolution_target}, match: {metrics.resolution_match}")

        return metrics

    def _analyze_color_quality(self, images: List[np.ndarray],
                             metrics: ImageQualityMetrics) -> ImageQualityMetrics:
        """Analyze color quality"""
        color_scores = []
        delta_e_values = []
        wb_scores = []
        saturation_scores = []

        for image in images:
            # Convert to LAB color space for analysis
            lab_image = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Color accuracy evaluation
            color_score = self._evaluate_color_accuracy(image, lab_image)
            color_scores.append(color_score)

            # Delta E calculation (simplified version)
            delta_e = self._calculate_delta_e(lab_image)
            delta_e_values.append(delta_e)

            # White balance evaluation
            wb_score = self._evaluate_white_balance(image)
            wb_scores.append(wb_score)

            # Saturation evaluation
            saturation_score = self._evaluate_saturation(hsv_image)
            saturation_scores.append(saturation_score)

        # Statistical results
        metrics.color_accuracy_score = statistics.mean(color_scores)
        metrics.color_delta_e = statistics.mean(delta_e_values)
        metrics.white_balance_score = statistics.mean(wb_scores)
        metrics.saturation_score = statistics.mean(saturation_scores)

        self.logger.info(f"Color quality analysis: accuracy {metrics.color_accuracy_score:.2f}, "
                        f"Delta E {metrics.color_delta_e:.2f}")

        return metrics

    def _analyze_sharpness(self, images: List[np.ndarray],
                         metrics: ImageQualityMetrics) -> ImageQualityMetrics:
        """Analyze image sharpness"""
        sharpness_scores = []
        blur_scores = []
        edge_densities = []

        for image in images:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Laplacian variance method for sharpness measurement
            laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
            sharpness_score = min(100, laplacian_var / 100)  # Normalize to 0-100
            sharpness_scores.append(sharpness_score)

            # Blur detection
            blur_score = self._detect_blur(gray)
            blur_scores.append(blur_score)

            # Edge density
            edge_density = self._calculate_edge_density(gray)
            edge_densities.append(edge_density)

        metrics.sharpness_score = statistics.mean(sharpness_scores)
        metrics.blur_detection_score = statistics.mean(blur_scores)
        metrics.edge_density = statistics.mean(edge_densities)

        self.logger.info(f"Sharpness analysis: sharpness {metrics.sharpness_score:.2f}, "
                        f"edge density {metrics.edge_density:.2f}")

        return metrics

    def _analyze_exposure_contrast(self, images: List[np.ndarray],
                                 metrics: ImageQualityMetrics) -> ImageQualityMetrics:
        """Analyze exposure and contrast"""
        brightness_scores = []
        contrast_scores = []
        dynamic_ranges = []
        exposure_scores = []

        for image in images:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Brightness evaluation
            mean_brightness = np.mean(gray)
            brightness_score = self._evaluate_brightness(mean_brightness)
            brightness_scores.append(brightness_score)

            # Contrast evaluation
            contrast_score = self._evaluate_contrast(gray)
            contrast_scores.append(contrast_score)

            # Dynamic range
            dynamic_range = np.max(gray) - np.min(gray)
            dynamic_ranges.append(dynamic_range)

            # Exposure accuracy
            exposure_score = self._evaluate_exposure(gray)
            exposure_scores.append(exposure_score)

        metrics.brightness_score = statistics.mean(brightness_scores)
        metrics.contrast_score = statistics.mean(contrast_scores)
        metrics.dynamic_range = statistics.mean(dynamic_ranges)
        metrics.exposure_accuracy = statistics.mean(exposure_scores)

        self.logger.info(f"Exposure/contrast analysis: brightness {metrics.brightness_score:.2f}, "
                        f"contrast {metrics.contrast_score:.2f}")

        return metrics

    def _analyze_noise(self, images: List[np.ndarray],
                      metrics: ImageQualityMetrics) -> ImageQualityMetrics:
        """Analyze image noise"""
        noise_levels = []
        snr_values = []

        for image in images:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Noise level estimation
            noise_level = self._estimate_noise_level(gray)
            noise_levels.append(noise_level)

            # Signal-to-noise ratio calculation
            snr = self._calculate_snr(gray)
            snr_values.append(snr)

        metrics.noise_level = statistics.mean(noise_levels)
        metrics.snr_db = statistics.mean(snr_values)

        self.logger.info(f"Noise analysis: noise level {metrics.noise_level:.3f}, "
                        f"SNR {metrics.snr_db:.2f}dB")

        return metrics

    def _calculate_overall_quality(self, metrics: ImageQualityMetrics) -> ImageQualityMetrics:
        """Calculate overall quality score"""
        # Weight settings
        weights = {
            'resolution': 0.15,
            'color': 0.25,
            'sharpness': 0.25,
            'exposure': 0.20,
            'noise': 0.15
        }

        # Calculate individual scores
        resolution_score = 100 if metrics.resolution_match else 50
        color_score = (metrics.color_accuracy_score + metrics.white_balance_score) / 2
        sharpness_score = metrics.sharpness_score
        exposure_score = (metrics.brightness_score + metrics.contrast_score) / 2
        noise_score = max(0, 100 - metrics.noise_level * 100)

        # Weighted average
        overall_score = (
            resolution_score * weights['resolution'] +
            color_score * weights['color'] +
            sharpness_score * weights['sharpness'] +
            exposure_score * weights['exposure'] +
            noise_score * weights['noise']
        )

        metrics.overall_quality_score = overall_score

        # Quality grade
        if overall_score >= 90:
            metrics.quality_grade = "EXCELLENT"
        elif overall_score >= 80:
            metrics.quality_grade = "GOOD"
        elif overall_score >= 70:
            metrics.quality_grade = "ACCEPTABLE"
        elif overall_score >= 60:
            metrics.quality_grade = "POOR"
        else:
            metrics.quality_grade = "UNACCEPTABLE"

        return metrics

    # Helper methods
    def _generate_reference_patterns(self) -> Dict[str, np.ndarray]:
        """Generate reference patterns"""
        patterns = {}

        # Checkerboard pattern (for geometric correction and sharpness testing)
        patterns['checkerboard'] = np.zeros((480, 640), dtype=np.uint8)
        for i in range(0, 480, 40):
            for j in range(0, 640, 40):
                if (i//40 + j//40) % 2 == 0:
                    patterns['checkerboard'][i:i+40, j:j+40] = 255

        return patterns

    def _create_color_checker(self) -> np.ndarray:
        """Create color checker"""
        # Simplified color checker with standard colors
        colors = [
            [115, 82, 68],   # Dark brown
            [194, 150, 130], # Light brown
            [98, 122, 157],  # Blue
            [87, 108, 67],   # Green
            [133, 128, 177], # Purple
            [103, 189, 170], # Cyan
        ]

        checker = np.zeros((120, 360, 3), dtype=np.uint8)
        for i, color in enumerate(colors):
            x_start = i * 60
            checker[:, x_start:x_start+60] = color

        return checker

    def _evaluate_color_accuracy(self, image: np.ndarray, lab_image: np.ndarray) -> float:
        """Evaluate color accuracy"""
        # Simplified color accuracy evaluation
        # Calculate standard deviation of each channel as color richness indicator
        l_std = np.std(lab_image[:, :, 0])
        a_std = np.std(lab_image[:, :, 1])
        b_std = np.std(lab_image[:, :, 2])

        # Normalized score
        score = min(100, (l_std + a_std + b_std) / 3)
        return score

    def _calculate_delta_e(self, lab_image: np.ndarray) -> float:
        """Calculate Delta E (simplified version)"""
        # Calculate color difference from standard white point
        reference_white = [100, 0, 0]  # White point in LAB space

        # Take average of center region
        h, w = lab_image.shape[:2]
        center_region = lab_image[h//4:3*h//4, w//4:3*w//4]
        mean_lab = np.mean(center_region.reshape(-1, 3), axis=0)

        # Simplified Delta E calculation
        delta_e = np.sqrt(np.sum((mean_lab - reference_white) ** 2))
        return delta_e

    def _evaluate_white_balance(self, image: np.ndarray) -> float:
        """Evaluate white balance"""
        # Calculate mean ratio of RGB channels
        b_mean = np.mean(image[:, :, 0])
        g_mean = np.mean(image[:, :, 1])
        r_mean = np.mean(image[:, :, 2])

        # Ideally, RGB ratio should be close to 1:1:1
        ratio_rg = r_mean / (g_mean + 1e-6)
        ratio_bg = b_mean / (g_mean + 1e-6)

        # Calculate deviation from ideal ratio
        deviation = abs(ratio_rg - 1.0) + abs(ratio_bg - 1.0)
        score = max(0, 100 - deviation * 50)

        return score

    def _evaluate_saturation(self, hsv_image: np.ndarray) -> float:
        """Evaluate saturation"""
        saturation = hsv_image[:, :, 1]
        mean_saturation = np.mean(saturation)

        # Moderate saturation scores higher
        if mean_saturation < 50:
            score = mean_saturation * 2  # Saturation too low
        elif mean_saturation > 200:
            score = (255 - mean_saturation) * 2  # Saturation too high
        else:
            score = 100  # Moderate saturation

        return min(100, max(0, score))

    def _detect_blur(self, gray_image: np.ndarray) -> float:
        """Detect blur level"""
        # Use Sobel operator for edge detection
        sobelx = cv2.Sobel(gray_image, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(gray_image, cv2.CV_64F, 0, 1, ksize=3)
        edge_magnitude = np.sqrt(sobelx**2 + sobely**2)

        # High edge intensity indicates sharp image
        blur_score = np.mean(edge_magnitude)
        return min(100, blur_score / 100)

    def _calculate_edge_density(self, gray_image: np.ndarray) -> float:
        """Calculate edge density"""
        edges = cv2.Canny(gray_image, 50, 150)
        edge_pixels = np.sum(edges > 0)
        total_pixels = edges.size
        edge_density = edge_pixels / total_pixels
        return edge_density

    def _evaluate_brightness(self, mean_brightness: float) -> float:
        """Evaluate brightness"""
        # Ideal brightness is between 100-150
        if 100 <= mean_brightness <= 150:
            score = 100
        elif mean_brightness < 100:
            score = mean_brightness  # Too dark
        else:
            score = max(0, 200 - mean_brightness)  # Too bright

        return min(100, max(0, score))

    def _evaluate_contrast(self, gray_image: np.ndarray) -> float:
        """Evaluate contrast"""
        # Use standard deviation as contrast indicator
        contrast = np.std(gray_image)

        # Appropriate contrast should be between 40-80
        if 40 <= contrast <= 80:
            score = 100
        elif contrast < 40:
            score = contrast * 2.5  # Contrast too low
        else:
            score = max(0, 160 - contrast)  # Contrast too high

        return min(100, max(0, score))

    def _evaluate_exposure(self, gray_image: np.ndarray) -> float:
        """Evaluate exposure accuracy"""
        # Calculate histogram
        hist = cv2.calcHist([gray_image], [0], None, [256], [0, 256])
        hist = hist.flatten() / hist.sum()

        # Good exposure should be distributed across the full dynamic range
        # Check for overexposure (highlights) and underexposure (shadows)
        overexposed = np.sum(hist[240:])  # Overexposed pixel ratio
        underexposed = np.sum(hist[:15])  # Underexposed pixel ratio

        # Calculate exposure score
        exposure_penalty = (overexposed + underexposed) * 100
        score = max(0, 100 - exposure_penalty)

        return score

    def _estimate_noise_level(self, gray_image: np.ndarray) -> float:
        """Estimate noise level"""
        # Estimate noise using high-frequency components
        # Apply Gaussian filter then calculate difference
        blurred = cv2.GaussianBlur(gray_image, (5, 5), 0)
        noise = gray_image.astype(np.float32) - blurred.astype(np.float32)
        noise_level = np.std(noise) / 255.0  # Normalize

        return noise_level

    def _calculate_snr(self, gray_image: np.ndarray) -> float:
        """Calculate signal-to-noise ratio"""
        # Signal power (image variance)
        signal_power = np.var(gray_image)

        # Noise power estimation
        noise_level = self._estimate_noise_level(gray_image)
        noise_power = (noise_level * 255) ** 2

        # SNR calculation (dB)
        if noise_power > 0:
            snr_db = 10 * np.log10(signal_power / noise_power)
        else:
            snr_db = float('inf')

        return min(60, max(0, snr_db))  # Limit to reasonable range

    def evaluate_quality_thresholds(self, metrics: ImageQualityMetrics) -> Dict[str, Any]:
        """Evaluate image quality against thresholds"""
        thresholds = self.config.get("image_quality_thresholds", {})

        evaluation = {
            "overall_status": "PASS",
            "issues": [],
            "recommendations": [],
            "scores": {}
        }

        # Resolution check
        if not metrics.resolution_match:
            evaluation["issues"].append(
                f"Resolution mismatch: actual {metrics.resolution_actual}, target {metrics.resolution_target}"
            )

        # Color accuracy check
        max_delta_e = thresholds.get("max_color_delta_e", 5.0)
        if metrics.color_delta_e > max_delta_e:
            evaluation["issues"].append(f"Color deviation too large: Delta E={metrics.color_delta_e:.1f} > {max_delta_e}")

        # Sharpness check
        min_sharpness = thresholds.get("min_sharpness_score", 0.7) * 100
        if metrics.sharpness_score < min_sharpness:
            evaluation["overall_status"] = "FAIL"
            evaluation["issues"].append(f"Insufficient sharpness: {metrics.sharpness_score:.1f} < {min_sharpness}")

        # Contrast check
        min_contrast = thresholds.get("min_contrast_ratio", 0.3) * 100
        if metrics.contrast_score < min_contrast:
            evaluation["issues"].append(f"Contrast too low: {metrics.contrast_score:.1f} < {min_contrast}")

        # Noise check
        max_noise = thresholds.get("max_noise_level", 0.1)
        if metrics.noise_level > max_noise:
            evaluation["issues"].append(f"Noise too high: {metrics.noise_level:.3f} > {max_noise}")

        # Generate recommendations
        if metrics.overall_quality_score < 70:
            evaluation["recommendations"].extend([
                "Check if camera lens is clean",
                "Adjust lighting conditions",
                "Verify camera focus settings"
            ])

        if metrics.color_delta_e > 3:
            evaluation["recommendations"].append("Adjust white balance settings")

        if metrics.noise_level > 0.05:
            evaluation["recommendations"].append("Lower ISO or improve lighting conditions")

        evaluation["scores"] = {
            "resolution": 100 if metrics.resolution_match else 0,
            "color": metrics.color_accuracy_score,
            "sharpness": metrics.sharpness_score,
            "exposure": metrics.brightness_score,
            "overall": metrics.overall_quality_score
        }

        return evaluation

# Test function
def test_image_quality_analyzer():
    """Test image quality analyzer"""
    import logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    # Create camera configuration
    with CameraConfig() as camera_config:
        if not camera_config.initialize_camera():
            print("Camera initialization failed, unable to perform image quality analysis")
            return

        # Create image quality analyzer
        analyzer = ImageQualityAnalyzer(camera_config)

        # Run image quality analysis
        print("Running image quality analysis...")
        metrics = analyzer.analyze_image_quality(sample_count=10)

        # Evaluate quality
        evaluation = analyzer.evaluate_quality_thresholds(metrics)

        print(f"\nImage quality analysis results:")
        print(f"Resolution: {metrics.resolution_actual} (match: {metrics.resolution_match})")
        print(f"Overall quality score: {metrics.overall_quality_score:.2f} ({metrics.quality_grade})")
        print(f"Sharpness: {metrics.sharpness_score:.2f}")
        print(f"Color accuracy: {metrics.color_accuracy_score:.2f}")
        print(f"Noise level: {metrics.noise_level:.3f}")
        print(f"Evaluation status: {evaluation['overall_status']}")

if __name__ == "__main__":
    test_image_quality_analyzer()
