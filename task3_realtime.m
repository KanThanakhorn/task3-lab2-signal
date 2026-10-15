%% 
% ==============================================================================
% TASK 3: Real-Time Audio Sampling Visualization & FFT Analysis
% Platform: Altera MAX 10 FPGA (DE10-Lite) + ESP32-S3 + MATLAB R2026a
% ==============================================================================
% Parameters:
%   Sampling Rate (Fs)      : 16,000 Sa/sec (16 kS/s)
%   Block Size (N)          : 1024 samples
%   Frequency Resolution    : Fs / N = 16000 / 1024 = 15.625 Hz
%   Nyquist Frequency       : 8,000 Hz
%   Baud Rate               : 921,600 bps
%   Voltage Range           : 0.0 V - 5.0 V (DE10-Lite Arduino A0 front-end)
% ==============================================================================

clear serialport; clear; close all; clc;

%% 1. Configuration
baudRate    = 921600;
Fs          = 16000;       % 16 kS/s
N           = 1024;        % Block size
PACKET_SIZE = 4 + 2 + (N * 2); % 2054 bytes total per packet

% Time and frequency axes
dt = 1 / Fs;
t  = (0 : N-1) * dt * 1000;              % Time in milliseconds (0 to 64 ms)
f  = (0 : (N/2)-1) * (Fs / N);            % Frequency axis (0 to 8000 Hz, step 15.625 Hz)
hannWin = hann(N)';                      % Hann window (1 x N row vector)

%% 2. Connect to Serial Port
availablePorts = serialportlist();
if isempty(availablePorts)
    error('No serial (COM) port found! Please plug in ESP32-S3 via USB.');
end

if ismember("COM7", availablePorts)
    portName = "COM7";
else
    portName = availablePorts(end);
end

fprintf('Connecting to %s at %d baud...\n', portName, baudRate);

try
    s = serialport(portName, baudRate, "Timeout", 1);
    flush(s);
catch ME
    error('Failed to open %s: %s\nMake sure other software (e.g. Serial Monitor) is closed!', portName, ME.message);
end

%% 3. Setup Interactive GUI Figure
fig = figure('Name', 'Task 3: MAX 10 FPGA Audio Sampling (16 kS/s)', ...
             'NumberTitle', 'off', 'Color', [0.15 0.15 0.18], ...
             'Position', [100, 100, 1100, 750]);

% Subplot 1: Time Domain Waveform
ax1 = subplot(3, 1, 1, 'Parent', fig);
hWave = plot(ax1, t, repmat(2.5, 1, N), 'Color', [0.0 0.8 1.0], 'LineWidth', 1.5);
grid(ax1, 'on');
title(ax1, 'Time-Domain Audio Waveform (MAX 10 On-Chip ADC @ 16 kS/s)', ...
      'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
xlabel(ax1, 'Time (ms)', 'Color', 'w');
ylabel(ax1, 'Voltage (V)', 'Color', 'w');
ylim(ax1, [0.0 5.0]);
xlim(ax1, [0 t(end)]);
set(ax1, 'Color', [0.1 0.1 0.12], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);

% Subplot 2: FFT Magnitude Spectrum
ax2 = subplot(3, 1, 2, 'Parent', fig);
hFFT_rect = plot(ax2, f, zeros(1, N/2), '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.0); hold(ax2, 'on');
hFFT_hann = plot(ax2, f, zeros(1, N/2), 'Color', [1.0 0.4 0.2], 'LineWidth', 1.8);
grid(ax2, 'on');
title(ax2, 'FFT Magnitude Spectrum (\Delta f = 15.625 Hz, Nyquist = 8 kHz)', ...
      'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
xlabel(ax2, 'Frequency (Hz)', 'Color', 'w');
ylabel(ax2, 'Magnitude (V)', 'Color', 'w');
xlim(ax2, [0 Fs/2]);
ylim(ax2, [0 1.5]);
legend(ax2, {'Without Window (Rectangular)', 'With Hann Window'}, ...
       'TextColor', 'w', 'Color', [0.2 0.2 0.2], 'Location', 'northeast');
set(ax2, 'Color', [0.1 0.1 0.12], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);

% Peak Frequency Annotation
hTextPeak = text(ax2, 0.03, 0.85, 'Peak: 0.0 Hz', 'Units', 'normalized', ...
                 'Color', [1.0 1.0 0.2], 'FontSize', 11, 'FontWeight', 'bold');

% Subplot 3: Real-Time Spectrogram
ax3 = subplot(3, 1, 3, 'Parent', fig);
numHistory = 60;
spectrogramBuffer = zeros(N/2, numHistory);
hSpec = imagesc(ax3, 1:numHistory, f, spectrogramBuffer);
axis(ax3, 'xy');
colormap(ax3, 'jet');
title(ax3, 'Real-Time Audio Spectrogram / Waterfall', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
xlabel(ax3, 'Buffer Block Frames', 'Color', 'w');
ylabel(ax3, 'Frequency (Hz)', 'Color', 'w');
set(ax3, 'Color', [0.1 0.1 0.12], 'XColor', 'w', 'YColor', 'w');
cb = colorbar(ax3);
cb.Color = 'w';
ylabel(cb, 'Magnitude (dB)', 'Color', 'w');

fprintf('Receiving real-time stream from FPGA... Speak into the microphone!\n');

%% 4. Bulletproof Stream Processing Loop
rawStreamBuf = uint8([]);
MAGIC_HDR    = uint8([hex2dec('55'), hex2dec('AA'), hex2dec('55'), hex2dec('AA')]);
frameCount   = 0;

while ishandle(fig)
    try
        % Read any incoming bytes into our stream buffer
        nAvail = s.NumBytesAvailable;
        if nAvail > 0
            newBytes = read(s, nAvail, "uint8");
            rawStreamBuf = [rawStreamBuf, newBytes]; %#ok<AGROW>
        else
            pause(0.005);
            continue;
        end

        % Discard stale data if buffer grew excessively (> 6000 bytes)
        if length(rawStreamBuf) > 6000
            rawStreamBuf = rawStreamBuf(end-3000:end);
        end

        % Search for Magic Header: 55 AA 55 AA
        idx = strfind(rawStreamBuf, MAGIC_HDR);
        if isempty(idx)
            continue;
        end

        % Align buffer to the earliest detected header
        if idx(1) > 1
            rawStreamBuf(1 : idx(1) - 1) = [];
        end

        % Check if full packet has arrived
        if length(rawStreamBuf) < PACKET_SIZE
            continue;
        end

        % If buffer has accumulated backlog of frames, fast-forward to newest
        while length(rawStreamBuf) >= (PACKET_SIZE * 2)
            idx_next = strfind(rawStreamBuf(PACKET_SIZE+1:end), MAGIC_HDR);
            if ~isempty(idx_next)
                rawStreamBuf(1 : PACKET_SIZE) = [];
            else
                break;
            end
        end

        % Extract complete packet payload
        packetBytes = rawStreamBuf(1 : PACKET_SIZE);
        rawStreamBuf(1 : PACKET_SIZE) = [];

        % Verify block size N = 1024 (bytes 5 and 6)
        blockSize = typecast(uint8(packetBytes(5:6)), 'uint16');
        if blockSize ~= N
            continue;
        end

        % Extract 1024 samples (bytes 7 to 2054)
        sampleBytes = packetBytes(7:end);
        rawSamples  = typecast(uint8(sampleBytes), 'uint16');
        frameCount  = frameCount + 1;

        % Convert 12-bit ADC raw integer (0..4095) to Voltage (0..5V)
        voltage = (double(rawSamples(:)') / 4095.0) * 5.0; % 1 x N row vector

        % Compute AC component
        dcOffset  = mean(voltage);
        dcRemoved = voltage - dcOffset; % 1 x N row vector

        % 1. FFT without windowing
        Y_rect = abs(fft(dcRemoved, N)) / (N / 2);
        Y_rect_single = Y_rect(1 : N/2); % 1 x (N/2) row vector

        % 2. FFT with Hann windowing
        windowedSignal = dcRemoved .* hannWin; % (1 x N) .* (1 x N) = 1 x N
        Y_hann = abs(fft(windowedSignal, N)) / (sum(hannWin) / 2);
        Y_hann_single = Y_hann(1 : N/2); % 1 x (N/2) row vector

        % Find dominant peak frequency (ignore DC bin)
        [maxMag, maxIdx] = max(Y_hann_single(2:end));
        peakFreq = f(maxIdx + 1);

        % Update Spectrogram
        mag_dB = 20 * log10(Y_hann_single(:) + 1e-6); % (N/2) x 1 column vector
        spectrogramBuffer = [spectrogramBuffer(:, 2:end), mag_dB]; % (N/2) x 60

        % Update Plots
        set(hWave, 'YData', voltage);
        set(hFFT_rect, 'YData', Y_rect_single);
        set(hFFT_hann, 'YData', Y_hann_single);
        set(hSpec, 'CData', spectrogramBuffer);

        if dcOffset > 4.8
            statusStr = sprintf('Dominant Peak: %.1f Hz | Mean V: %.2fV (Check Mic OUT -> A0 connection)', peakFreq, dcOffset);
        else
            statusStr = sprintf('Dominant Peak: %.1f Hz (Mag: %.3f V) | DC Offset: %.2f V | Frames: %d', peakFreq, maxMag, dcOffset, frameCount);
        end
        set(hTextPeak, 'String', statusStr);

        if maxMag > 0.05
            ylim(ax2, [0, maxMag * 1.3]);
        else
            ylim(ax2, [0, 0.2]);
        end

        drawnow limitrate;

    catch ME
        % Do not crash on minor serial hiccups
        fprintf('Recovering from frame glitch: %s\n', ME.message);
        pause(0.01);
    end
end

clear s;
disp('MATLAB visualization terminated gracefully.');
