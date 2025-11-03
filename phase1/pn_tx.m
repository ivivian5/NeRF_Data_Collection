%% -----------------------------
% PN-Sequence Transmitter for Channel Sounding
%% -----------------------------

centerFrequency = 9.15e8;    % 915 MHz
masterClock = 30e6;
interpFactor = 60;           % gives 500 kHz baseband sample rate
txGain = 45;                 % adjust to avoid clipping or saturation

% --- Generate PN sequence ---
pnLen = 1023;   % length of one sequence (Gold or maximal)
pn = comm.PNSequence('Polynomial',[10 3 0],'SamplesPerFrame',pnLen);
seq = 2*double(pn()) - 1;    % map {0,1} -> {-1,+1}

% Repeat and upsample to continuous stream
seqUpsampled = repmat(seq, 50, 1);
seqComplex = complex(seqUpsampled, 0);  % real BPSK baseband

% --- Create transmitter object ---
tx = comm.SDRuTransmitter( ...
    'Platform','B210', ...
    'SerialNum','31AB6D1', ...  % <-- change to your TX serial
    'CenterFrequency', centerFrequency, ...
    'Gain', txGain, ...
    'MasterClockRate', masterClock, ...
    'InterpolationFactor', interpFactor, ...
    'ChannelMapping', 1);

sampleRate = tx.MasterClockRate / tx.InterpolationFactor;
disp(['TX sample rate: ', num2str(sampleRate/1e3), ' kHz']);

% --- Transmit continuously ---
disp('Transmitting PN sequence continuously... Press Ctrl+C to stop.');
while true
    tx(seqComplex);
end

release(tx);
