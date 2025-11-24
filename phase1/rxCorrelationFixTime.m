%% ------------ RECEIVE FROM USRP AND STORE INTO rxIQ -----------------

% --- Receiver parameters ---
centerFrequency = 915e6;
masterClock = 30e6;
decim = 60;
gain = 61;         % adjust as needed
numSeconds = 20;    % how long to record

rx = comm.SDRuReceiver( ...
    'Platform', 'B210', ...
    'SerialNum', '34C78FD', ...
    'MasterClockRate', masterClock, ...
    'DecimationFactor', decim, ...
    'CenterFrequency', centerFrequency, ...
    'ChannelMapping', 1, ...         % *** IMPORTANT: use only one channel ***
    'Gain', gain, ...
    'OutputDataType', 'double');

fprintf('Sample rate = %.3f kHz\n', rx.MasterClockRate/rx.DecimationFactor/1e3);

% ---- Capture samples ----
[data, metadata] = capture(rx, numSeconds, "Seconds");
release(rx);

% data is complex baseband IQ
rxIQ = data(:,1);
rxIQ = complex(real(data(:,1)), imag(data(:,1))); 

% sample rate
fs = rx.MasterClockRate / rx.DecimationFactor;
fprintf('Captured %.1f million samples\n', length(rxIQ)/1e6);

rx = rxIQ;

%% -----------------------------
% Receiver: detect shaped PN preamble, correct CFO, extract payload
% Assumptions:
%  - rx is a column vector of complex baseband samples (from capture)
%  - Fs, Rsym, sps, seed, numSymsPayload, qpskMod, qpskDem, rrc are known and same as TX
% ------------------------------

% --- Parameters (must match transmitter) ---
Fs = 500e3;            % sample rate (Hz)
seed = 12345;
numSymsPayload = 2000; % same as TX

M = 4;
bitsPerSym = log2(M);
Rsym = 100e3; % Symbol rate 100 ksps
sps = Fs / Rsym; % samples per symbol = 5
qpskMod = comm.QPSKModulator(...
BitInput=true, ...
PhaseOffset=pi/4); % Gray-coded QPSK
rrc = comm.RaisedCosineTransmitFilter(...
RolloffFactor=0.25, ...
FilterSpanInSymbols=6, ...
OutputSamplesPerSymbol=sps);

% --- regenerate PN preamble & shaped preamble (exactly like TX) ---
pn = comm.PNSequence('Polynomial',[7 0], ...
                     'InitialConditions',ones(1,7), ...
                     'SamplesPerFrame',127);
preamble_bits = pn();

% QPSK mod (matches TX)
if mod(length(preamble_bits),2) ~= 0
    preamble_bits = [preamble_bits; 0];
end
qpskMod = comm.QPSKModulator('BitInput',true);
preamble_sym = qpskMod(preamble_bits);

% Your TX RRC filter
rrc = comm.RaisedCosineTransmitFilter( ...
    'RolloffFactor',0.35, ...
    'FilterSpanInSymbols',8, ...
    'OutputSamplesPerSymbol',sps);

preamble_ref = rrc(preamble_sym);
preamble_ref = preamble_ref / max(abs(preamble_ref));

lenPre = length(preamble_ref);


%% =============================
% 2) FIND THE PACKET START
% =============================
[c, lags] = xcorr(rx, preamble_ref);
[~, idx] = max(abs(c));
lag_at_peak = lags(idx);
startIndex = lag_at_peak + 1;

fprintf("Packet start index = %d\n", startIndex);


%% =============================
% 3) CUT THE RX SIGNAL AFTER PREAMBLE
% =============================
if startIndex + lenPre > length(rx)
    error("Preamble does not fully fit in the capture");
end

rx_aligned = rx(startIndex + lenPre : end);

fprintf("Aligned IQ length: %d samples\n", length(rx_aligned));
