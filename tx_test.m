tx = comm.SDRuTransmitter(Platform="B210",...
              SerialNum='34C78FD',...
              MasterClockRate=60e6,...
              InterpolationFactor=60,...
              CenterFrequency=0.915e9,Gain=20,...
              TransportDataType="int16");

sinewave = dsp.SineWave(2,40e3);
sinewave.SampleRate = 30e6/60;
sinewave.SamplesPerFrame = 5e4;
sinewave.OutputDataType = 'double';
sinewave.ComplexOutput = true;
data = sinewave();

amp = max(abs(data));
NormalizedData = data/amp;

spectrumScope = spectrumAnalyzer('SampleRate',sinewave.SampleRate);
stoptime = 30;
frameDuration = (sinewave.SamplesPerFrame)/(sinewave.SampleRate);
time = 0;
timeScope = timescope('TimeSpan',4/40e3,'SampleRate', 30e6/60);
disp("Transmission started");

spectrumScope(NormalizedData);
timeScope(NormalizedData);

while time < stoptime
    if tx(NormalizedData)
        warning("Underrun detected");
    end
    time = time+frameDuration;
end

disp("Transmission end");
release(tx);


radio = comm.SDRuTransmitter(Platform ="B210",SerialNum ="34C78FD");
radio.CenterFrequency = 0.915e9;
radio.LocalOscillatorOffset = 1000;
radio.Gain = 8.3;
radio.MasterClockRate = 10.56789e6;
radio.InterpolationFactor = 510;
info(radio)                   
