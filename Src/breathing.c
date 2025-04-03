#include <stdio.h>
#include <math.h>

#define WINDOW_SIZE 5
#define SAMPLING_RATE 2
#define SECONDS_PER_MINUTE 60

void smooth_data(float *data, int length, float *smoothed) {
		// compute simple moving average with sliding window
    for (int i = 0; i < length; i++) {
        float sum = 0;
        int count = 0;
        for (int j = i - WINDOW_SIZE / 2; j <= i + WINDOW_SIZE / 2; j++) {
            if (j >= 0 && j < length) {
                sum += data[j];
                count++;
            }
        }
        smoothed[i] = sum / count;
    }
}


// since we're sampling discretely at not a high rate (for power-saving)
// peaks would occur within a 3 sample slice, which is what we're checking here
int count_peaks(float *data, int length) {
    int peaks = 0;
    for (int i = 1; i < length - 1; i++) {
        if (data[i] > data[i - 1] && data[i] > data[i + 1]) {
            peaks++;
        }
    }
    return peaks;
}

float estimate_breathing_rate(float *vertical_accel, int length) {
    float smoothed[length];
    smooth_data(vertical_accel, length, smoothed);
    
    int peak_count = count_peaks(smoothed, length);
    float duration_seconds = (float)length / SAMPLING_RATE;
    
	// BrPM formula
    return (peak_count * SECONDS_PER_MINUTE) / duration_seconds;
}
