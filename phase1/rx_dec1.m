clear all;
%% ==============================================================================
%  NeRF2 RECEIVER (Single Capture)
% ==============================================================================

% 1. GENERATE REFERENCE (Must match TX exactly)
Fs = 500e3;
pn = comm.PNSequence('Polynomial', [10 3 0], 'InitialConditions', ones(1,10), 'SamplesPerFrame', 1023);
ref_bits = pn();
if mod(length(ref_bits), 2) ~= 0, ref_bits = [ref_bits; 0]; end
qpskMod = comm.QPSKModulator('BitInput',true, 'PhaseOffset', pi/4);
ref_syms = qpsskMod(ref_bits);
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
    'Gain',                 60, ...          % FIXED GAIN
    'ChannelMapping',       [1 2], ...       % 2 Antennas
    'OutputDataType',       'double');

% 3. PROCESSING OBJECTS
cfoObj = comm.CoarseFrequencyCompensator('Modulation','QPSK','SampleRate',Fs, 'FrequencyResolution', 50);

disp("Listening for a single packet...");
figure(1); clf;

rxBuffer = complex(zeros(0, 2)); % 2-column buffer
maxBuffer = 200000; % Max size to prevent memory hog

while true
    % A. Receive Data
    [rxChunk, len, overrun] = rxObj();
    if overrun, disp("Overrun"); end

    rxBuffer = [rxBuffer; rxChunk];

    % Trim buffer to prevent overflow
    if size(rxBuffer, 1) > maxBuffer
        rxBuffer = rxBuffer(end - maxBuffer + 1 : end, :);
    end

    % B. Detection (Cross-Correlate Channel 1)
    if size(rxBuffer, 1) > N + 5000
        [c, lags] = xcorr(rxBuffer(:,1), reference_sig);
        [maxVal, idx] = max(abs(c));

        % THRESHOLD CHECK (Adjust based on your measured peak)
        if maxVal > 15 
            lag = lags(idx);
            startIdx = lag + 1;

            % Ensure we have the full packet 
            if startIdx > 0 && (startIdx + N < size(rxBuffer, 1))
                
                % C. EXTRACT RAW IQ (Both Antennas)
                packet_ant1 = rxBuffer(startIdx : startIdx + N - 1, 1);
                packet_ant2 = rxBuffer(startIdx : startIdx + N - 1, 2);
                
                % D. CFO CORRECTION
                [~, estCFO] = cfoObj(packet_ant1);
                t = (0:N-1).' / Fs;
                cfo_vector = exp(-1i * 2 * pi * estCFO * t);
                
                clean_ant1 = packet_ant1 .* cfo_vector;
                clean_ant2 = packet_ant2 .* cfo_vector;
                
                % E. VISUALIZATION
                subplot(2,2,1); plot(lags, abs(c)); title('Correlation Peak');
                subplot(2,2,2); plot(abs(clean_ant1)); title('Packet Envelope (Ant 1)');
                % Only sample every 10th sample to avoid plotting too many points
                subplot(2,2,3); scatterplot(clean_ant1(1:10:end), 1, 0, 'b.'); title('Constellation (CFO Corrected)'); 
                subplot(2,2,4); scatterplot(clean_ant2(1:10:end), 1, 0, 'r.'); title('Constellation (Ant 2)'); 
                drawnow;

                % F. SAVE FINAL DATA STRUCTURE
                final_data.clean_ant1 = clean_ant1;
                final_data.clean_ant2 = clean_ant2;
                final_data.cfo = estCFO;
                final_data.timestamp = datetime('now');
                
                save('nerf_single_capture.mat', 'final_data');
                
                fprintf("\n✅ First packet captured and saved to nerf_single_capture.mat\n");
                
                % G. TERMINATE SCRIPT
                break;
            end
        end
    end
end

release(rxObj);
disp("Done.");
