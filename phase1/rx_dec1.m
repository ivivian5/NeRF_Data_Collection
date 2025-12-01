clear all;
%% ==============================================================================
%  NeRF2 RECEIVER (Optimized Single Capture)
% ==============================================================================

% 1. REFERENCE AND CONFIG
Fs = 500e3;
ChunkSize = 10000; % Set a large, fixed chunk size for reading
N = 5115; % Calculated length of reference_sig (1024 symbols * 5 sps - 5 samples)

% (Code to generate reference_sig remains identical)
pn = comm.PNSequence('Polynomial', [10 3 0], 'InitialConditions', ones(1,10), 'SamplesPerFrame', 1023);
ref_bits = pn();
if mod(length(ref_bits), 2) ~= 0, ref_bits = [ref_bits; 0]; end
qpskMod = comm.QPSKModulator('BitInput',true, 'PhaseOffset', pi/4);
ref_syms = qpskMod(ref_bits);
sps = 5;
rrcTx = comm.RaisedCosineTransmitFilter('RolloffFactor', 0.25, 'FilterSpanInSymbols', 6, 'OutputSamplesPerSymbol', sps);
reference_sig = rrcTx(ref_syms);
reference_sig = reference_sig / max(abs(reference_sig));
N = length(reference_sig); % Corrected N = 5115

% 2. HARDWARE CONFIG (WITH SamplesPerFrame)
rxObj = comm.SDRuReceiver(...
    'Platform',             'B210', ...
    'SerialNum',            '34C78FD', ...
    'MasterClockRate',      30e6, ...
    'DecimationFactor',     60, ...
    'SamplesPerFrame',      ChunkSize, ...
    'CenterFrequency',      915e6, ...
    'Gain',                 60, ...
    'ChannelMapping',       [1 2], ...
    'OutputDataType',       'double');

% 3. PROCESSING OBJECTS
cfoObj = comm.CoarseFrequencyCompensator('Modulation','QPSK','SampleRate',Fs, 'FrequencyResolution', 50);

disp("Listening for a single packet...");
figure(1); clf;

rxBuffer = complex(zeros(0, 2));
minBufferForCheck = N + ChunkSize; % Only correlate when we have enough data to cover one packet + one chunk

while true
    % A. Receive Data
    % rxChunk will always be ChunkSize x 2
    [rxChunk, ~, overrun] = rxObj();
    if overrun
        warning('Overrun detected! Increase SamplesPerFrame or reduce processing load.');
    end
    
    rxBuffer = [rxBuffer; rxChunk];
    
    % B. Detection (Correlate only if enough data is present)
    if size(rxBuffer, 1) >= minBufferForCheck
        
        % C. Use a sliding correlation window for efficiency 
        % Correlate only the most recent 'minBufferForCheck' samples, not the entire historical buffer
        checkWindow = rxBuffer(end - minBufferForCheck + 1 : end, 1);
        [c, lags] = xcorr(checkWindow, reference_sig);
        
        [maxVal, idx] = max(abs(c));
        
        % THRESHOLD CHECK
        if maxVal > 15 
            
            % Since we correlated on the checkWindow, we need to map the lag back to the main buffer
            lagInWindow = lags(idx);
            startIdx = size(rxBuffer, 1) - minBufferForCheck + 1 + lagInWindow;
            
            % D. Extraction and Processing (Only if bounds are safe)
            if startIdx > 0 && (startIdx + N < size(rxBuffer, 1))
                
                packet_ant1 = rxBuffer(startIdx : startIdx + N - 1, 1);
                packet_ant2 = rxBuffer(startIdx : startIdx + N - 1, 2);
                
                % CFO Correction
                [~, estCFO] = cfoObj(packet_ant1);
                t = (0:N-1).' / Fs;
                cfo_vector = exp(-1i * 2 * pi * estCFO * t);
                
                clean_ant1 = packet_ant1 .* cfo_vector;
                clean_ant2 = packet_ant2 .* cfo_vector;
                
                % E. VISUALIZATION AND SAVE
                subplot(2,2,1); plot(lags, abs(c)); title(['Correlation Peak (Max:', num2str(maxVal, '%.2f'), ')']);
                subplot(2,2,2); plot(abs(clean_ant1)); title('Packet Envelope (Ant 1)');
                subplot(2,2,3); scatterplot(clean_ant1(1:10:end), 1, 0, 'b.'); title(['Constellation (CFO:', num2str(estCFO, '%.1f'), ' Hz)']);
                subplot(2,2,4); scatterplot(clean_ant2(1:10:end), 1, 0, 'r.'); title('Constellation (Ant 2)');
                drawnow;

                final_data.clean_ant1 = clean_ant1;
                final_data.clean_ant2 = clean_ant2;
                final_data.cfo = estCFO;
                
                save('nerf_single_capture.mat', 'final_data');
                
                fprintf("\n✅ First packet captured and saved.\n");
                
                % F. TERMINATE SCRIPT
                break; 
            end
        end
        
        % G. Aggressively trim the old buffer if no packet was found in the latest chunk
        % Keep only the last full check window to look for the next chunk's packet
        rxBuffer = rxBuffer(end - minBufferForCheck + 1 : end, :);

    end
end

release(rxObj);
disp("Done.");
