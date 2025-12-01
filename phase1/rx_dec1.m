clear all;
%% ==============================================================================
%  NeRF2 RECEIVER (Data Collector)
% ==============================================================================

% 1. GENERATE REFERENCE (Must match TX exactly)
Fs = 500e3; 
pn = comm.PNSequence('Polynomial', [10 3 0], 'InitialConditions', ones(1,10), 'SamplesPerFrame', 1023);
ref_bits = pn();
if mod(length(ref_bits), 2) ~= 0, ref_bits = [ref_bits; 0]; end
qpskMod = comm.QPSKModulator('BitInput',true, 'PhaseOffset', pi/4);
ref_syms = qpskMod(ref_bits);
sps = 5;
rrcTx = comm.RaisedCosineTransmitFilter('RolloffFactor', 0.25, 'FilterSpanInSymbols', 6, 'OutputSamplesPerSymbol', sps);
reference_sig = rrcTx(ref_syms);
% Normalize reference for correlation
reference_sig = reference_sig / max(abs(reference_sig));
N = length(reference_sig);

% 2. HARDWARE CONFIG
rxObj = comm.SDRuReceiver(...
    'Platform',             'B210', ...
    'SerialNum',            '34C78FD', ...
    'MasterClockRate',      30e6, ...
    'DecimationFactor',     60, ...
    'CenterFrequency',      915e6, ...
    'Gain',                 60, ...          % MUST MATCH TX GAIN roughly, keep FIXED
    'ChannelMapping',       [1 2], ...       % 2 Antennas
    'OutputDataType',       'double');

% 3. PROCESSING OBJECTS
% Freq correction (Hardware drift removal only)
cfoObj = comm.CoarseFrequencyCompensator('Modulation','QPSK','SampleRate',Fs, 'FrequencyResolution', 50);

disp("Starting Data Collection...");
figure(1); clf;

packetCount = 0;
maxPackets = 100; % Stop after this many
dataStorage = {}; % Cell array to store packets

rxBuffer = complex(zeros(0, 2)); % 2-column buffer

while packetCount < maxPackets
    % A. Receive Data
    [rxChunk, len, overrun] = rxObj();
    if overrun, disp("Overrun"); end
    
    rxBuffer = [rxBuffer; rxChunk];
    
    % B. Detection (Cross-Correlate Channel 1)
    % Only process if we have enough data
    if size(rxBuffer, 1) > N + 5000
        [c, lags] = xcorr(rxBuffer(:,1), reference_sig);
        [maxVal, idx] = max(abs(c));
        
        % THRESHOLD CHECK (Adjust based on noise floor)
        if maxVal > 10 
            lag = lags(idx);
            startIdx = lag + 1;
            
            % Ensure we have the full packet + some margin
            if startIdx > 0 && (startIdx + N < size(rxBuffer, 1))
                
                % C. EXTRACT RAW IQ (Both Antennas)
                packet_ant1 = rxBuffer(startIdx : startIdx + N - 1, 1);
                packet_ant2 = rxBuffer(startIdx : startIdx + N - 1, 2);
                
                % D. CFO CORRECTION
                % We calculate CFO on Ant1 and apply it to BOTH.
                % This preserves the relative phase difference between antennas.
                [~, estCFO] = cfoObj(packet_ant1);
                
                t = (0:N-1).' / Fs;
                cfo_vector = exp(-1i * 2 * pi * estCFO * t);
                
                clean_ant1 = packet_ant1 .* cfo_vector;
                clean_ant2 = packet_ant2 .* cfo_vector;
                
                % E. SAVE DATA
                packetCount = packetCount + 1;
                timestamp = now;
                
                % Store structure
                pktStruct.id = packetCount;
                pktStruct.cfo = estCFO;
                pktStruct.peak = maxVal;
                pktStruct.data = [clean_ant1, clean_ant2]; % N x 2 Matrix
                dataStorage{end+1} = pktStruct;
                
                fprintf("Packet %d Captured. CFO: %.2f Hz. Peak: %.2f\n", packetCount, estCFO, maxVal);
                
                % F. VISUALIZATION (Verify Integrity)
                subplot(2,2,1); plot(abs(c)); title('Correlation Peak');
                subplot(2,2,2); plot(abs(clean_ant1)); title('Packet Envelope (Ant 1)');
                subplot(2,2,3); scatterplot(clean_ant1(1:10:end), 1, 0, 'b.'); title('Constellation (No Sync)');
                drawnow;
                
                % G. FLUSH BUFFER
                % Remove the processed data from buffer to look for next packet
                rxBuffer = rxBuffer(startIdx + N + 1000 : end, :);
            else
                % Peak detected but data not fully in buffer yet, wait for next loop
            end
        else
            % No peak, trim buffer to prevent overflow
            if size(rxBuffer,1) > 200000
                rxBuffer = rxBuffer(end-50000:end, :);
            end
        end
    end
end

% SAVE TO FILE
save('nerf_rf_data.mat', 'dataStorage');
disp("Data Collection Complete. Saved to nerf_rf_data.mat");
release(rxObj);
