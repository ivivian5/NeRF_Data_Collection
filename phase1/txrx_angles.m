%% -----------------------------
% USRP Receiver (Receive from TX/RX ports)
%% -----------------------------

centerFrequency = 9.15e8;
masterClock = 30e6;
decimationFactor = 60;
gain = 61;
captureTime = 5;

rx = comm.SDRuReceiver( ...
    'Platform', 'B210', ...
    'SerialNum', '34C78FD', ...
    'MasterClockRate', masterClock, ...
    'DecimationFactor', decimationFactor, ...
    'CenterFrequency', centerFrequency, ...
    'OutputDataType', 'double', ...
    'ChannelMapping', [1 2], ...
    'Gain', gain);

sampleRate = rx.MasterClockRate / rx.DecimationFactor;
disp(['Receiver sample rate: ', num2str(sampleRate/1e3), ' kHz']);

[data, metadata, timestamp] = capture(rx, captureTime, 'Seconds');
release(rx);

dataA = data(:,1);
dataB = data(:,2);

sA = spectrumAnalyzer('SampleRate', sampleRate, ...
                      'NumInputPorts', 2, ...
                      'ChannelNames', {'TX/RX A', 'TX/RX B'});
sA(real(dataA), real(dataB));
release(sA);

% --- AoA Estimation (MUSIC) ---
fc = centerFrequency;
lambda = physconst('LightSpeed') / fc;
d = lambda/2;
array = phased.ULA('NumElements', 2, 'ElementSpacing', d);

estimator = phased.MUSICEstimator('SensorArray', array, ...
    'OperatingFrequency', fc, ...
    'NumSignalsSource', 'Property', 'NumSignals', 1, ...
    'ScanAngles', -90:0.1:90);

angleSpectrum = estimator([dataA, dataB]);
[~, idx] = max(angleSpectrum);
aoa = estimator.ScanAngles(idx);
disp(['Estimated AoA: ', num2str(aoa), ' degrees']);
