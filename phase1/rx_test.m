clear all;

%% ===================== SDR PARAMETERS ================================
tx = comm.SDRuTransmitter(...
    Platform="B210", ...
    SerialNum='34C78EF', ...
    MasterClockRate=60e6, ...
    InterpolationFactor=60, ...     % 30e6 / 60 = 500 kS/s
    CenterFrequency=0.915e9, ...
    Gain=20, ...
    TransportDataType="int16");

Fs = 30e6 / 60;     % Baseband sample rate = 500 kHz
SamplesPerFrame = 50000;

%% ===================== QPSK + RRC PARAMETERS =========================
M = 4;
bitsPerSym = log2(M);
Rsym = 100e3;         % Symbol rate 100 ksps
sps = Fs / Rsym;      % samples per symbol = 5

qpskMod = comm.QPSKModulator(...
    BitInput=true, ...
    PhaseOffset=pi/4);        % Gray-coded QPSK

rrc = comm.RaisedCosineTransmitFilter(...
    RolloffFactor=0.25, ...
    FilterSpanInSymbols=6, ...
    OutputSamplesPerSymbol=sps);

%% ===================== PREAMBLE (PN SEQUENCE) ========================
% PN of length 127 (example)
pn = comm.PNSequence(...
    Polynomial=[7 3 0], ...   % degree-7 (x^7 + x^3 + 1)
    InitialConditions=ones(1,7), ...
    SamplesPerFrame=127);

preamble_bits = pn();   % 0/1 bits (127 long)

% QPSK requires even number of bits
if mod(length(preamble_bits), 2) ~= 0
    preamble_bits = [preamble_bits; 0];
end

preamble_symbols = qpskMod(preamble_bits);

%% ===================== PAYLOAD (DETERMINISTIC RNG) ===================
seed = 12345;
rng(seed, 'twister');

% choose payload length (in symbols)
numPayloadSymbols = 4000;           % adjust as needed
numPayloadBits = numPayloadSymbols * bitsPerSym;

payload_bits = randi([0 1], numPayloadBits, 1);
payload_symbols = qpskMod(payload_bits);

%% ===================== PACKET CONSTRUCTION ===========================
packet_symbols = [preamble_symbols; payload_symbols];

%% ===================== RRC SHAPING + NORMALIZATION ===================
tx_waveform = rrc(packet_symbols);

% Normalize
tx_waveform = tx_waveform / max(abs(tx_waveform));

%% ===================== DEBUG SCOPES =================================
spectrumScope = spectrumAnalyzer('SampleRate', Fs);
timeScope = timescope('TimeSpan', 5/Rsym, 'SampleRate', Fs);

disp("Showing initial waveform in scopes...");
spectrumScope(tx_waveform);
timeScope(tx_waveform);

%% ===================== TX LOOP ======================================
disp("Transmission started");

stoptime = 30;   % seconds
frameDuration = SamplesPerFrame / Fs;
time = 0;

% We need exactly SamplesPerFrame per call:
% Repeat waveform to fill frame
reps = ceil(SamplesPerFrame / length(tx_waveform));
tx_frame = repmat(tx_waveform, reps, 1);
tx_frame = tx_frame(1:SamplesPerFrame);

while time < stoptime

    underrun = tx(tx_frame);
    if underrun
        warning("Underrun detected!");
    end

    time = time + frameDuration;
end

disp("Transmission completed.");
release(tx);
