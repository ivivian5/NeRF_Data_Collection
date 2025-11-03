%% -----------------------------
% USRP Receiver for 40 kHz Sine Wave
%% -----------------------------

% --- Parameters (match transmitter) ---
centerFrequency = 9.15e8;           % 915 MHz RF
masterClock = 30e6;                 % same as transmitter
decimationFactor = 60;              % match your transmitter
gain = 61;                          

% --- Create the receiver object ---
rx = comm.SDRuReceiver( ...
    'Platform', 'B210', ...
    'SerialNum', '34C78EF', ...
    'MasterClockRate', masterClock, ...
    'DecimationFactor', decimationFactor, ...
    'CenterFrequency', centerFrequency, ...
    'OutputDataType', 'double', ...
    'Gain', gain);

sampleRate = rx.MasterClockRate / rx.DecimationFactor;  % Baseband sample rate
disp(['Receiver sample rate: ', num2str(sampleRate/1e3), ' kHz']);

% --- Capture 5 seconds of data ---
[data, metadata, timestamp] = capture(rx, 5, 'Seconds');
release(rx);

%% --- Optional: Bandpass filter around 40 kHz ---
fLow = 30e3;  % Hz
fHigh = 50e3; % Hz
bpFilt = designfilt('bandpassfir', ...
    'FilterOrder',64, ...
    'CutoffFrequency1', fLow, ...
    'CutoffFrequency2', fHigh, ...
    'SampleRate', sampleRate);

filteredData = filter(bpFilt, data);

%% --- Reconstruct sine wave ---
recoveredSine = real(filteredData);  % Take real part (can also use abs)

%% --- Time-domain visualization ---
t = (1.7:1.9)/sampleRate;
figure;
plot(t, recoveredSine);
xlabel('Time (s)'); ylabel('Amplitude');
title('Recovered 40 kHz Sine Wave');
grid on;

%% --- Frequency-domain visualization ---
figure;
spectrumAnalyzer = dsp.SpectrumAnalyzer('SampleRate', sampleRate);
spectrumAnalyzer(recoveredSine);
release(spectrumAnalyzer);

