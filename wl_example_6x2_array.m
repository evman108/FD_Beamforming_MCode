%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% wl_example_8x2_array.m
%
% Compatibility:
%   WARPLab:    v7.1.0 and later
%   Hardware:   v2 and v3
%
% Description:
%   See warpproject.org/trac/wiki/WARPLab/Examples/8x2Array
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all
figure(1);clf;

USE_AGC = true;
RUN_CONTINOUSLY = false;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set up the WARPLab experiment
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Create a vector of node objects
%This experiment uses 3 nodes: 2 will act as a transmitter and 1 will act
%as a receiver.
%   nodes(0): Primary transmitter
%   nodes(1): Secondary transmitter (receives clocks and triggers from
%             primary transmittter)
%   nodes(3): Receiver
nodes = wl_initNodes(3);

%Create a UDP broadcast trigger and tell each node to be ready for it
eth_trig = wl_trigger_eth_udp_broadcast;
wl_triggerManagerCmd(nodes,'add_ethernet_trigger',[eth_trig]);

%Read Trigger IDs into workspace
[T_IN_ETH,T_IN_ENERGY,T_IN_AGCDONE,T_IN_REG,T_IN_D0,T_IN_D1,T_IN_D2,T_IN_D3] =  wl_getTriggerInputIDs(nodes(1));
[T_OUT_BASEBAND, T_OUT_AGC, T_OUT_D0, T_OUT_D1, T_OUT_D2, T_OUT_D3] = wl_getTriggerOutputIDs(nodes(1));

%For the primary transmit node, we will allow Ethernet to trigger the buffer
%baseband, the AGC, and debug0 (which is mapped to pin 8 on the debug
%header). We also will allow Ethernet to trigger the same signals for the 
%receiving node.
% wl_triggerManagerCmd([nodes(1), nodes(3)],'output_config_input_selection', ...
%     [T_OUT_BASEBAND,T_OUT_AGC,T_OUT_D0, T_OUT_D1],[T_IN_ETH,T_IN_REG]);
wl_triggerManagerCmd([nodes(1)],'output_config_input_selection', ...
    [T_OUT_BASEBAND,T_OUT_AGC,T_OUT_D0, T_OUT_D1],[T_IN_ETH,T_IN_REG]);


wl_triggerManagerCmd([nodes(3)],'output_config_input_selection', ...
    [T_OUT_BASEBAND,T_OUT_AGC],[T_IN_D3]);

%For the secondary transmit node, we will allow debug3 (mapped to pin 15 on the
%debug header) to trigger the buffer baseband, and the AGC
wl_triggerManagerCmd(nodes(2),'output_config_input_selection', ...
    [T_OUT_BASEBAND,T_OUT_AGC],[T_IN_D3]);

%For the secondary node, we enable the debounce circuity on the debug 3 input
%to deal with the fact that the signal may be noisy.
wl_triggerManagerCmd(nodes(2), 'input_config_debounce_mode',[T_IN_D3],'enable'); 

%Since the debounce circuitry is enabled, there will be a delay at the
%receiver node for its input trigger. To better align the transmitter and
%receiver, we can artificially delay the transmitters trigger outputs that
%drive the buffer baseband and the AGC.
%
%NOTE:  Due to HW changes in WARPLab 7.2.0, the input delay of the trigger 
%manager increased by one clock cycle;  Therefore, when using WARPLab 7.2.0, 
%we need to increase the output delay by one step.  If using WARPLab 7.1.0, 
%please use the commented out line below:
%
%nodes(1).wl_triggerManagerCmd('output_config_delay',[T_OUT_BASEBAND,T_OUT_AGC],[50]); %50ns delay  - WARPLab 7.1.0
%
nodes(1).wl_triggerManagerCmd('output_config_delay',[T_OUT_BASEBAND,T_OUT_AGC],[56.25]); %56.25ns delay  - WARPLab 7.2.0

%Get IDs for the interfaces on the boards. Since this example assumes each
%board has the same interface capabilities, we only need to get the IDs
%from one of the boards
[RFA,RFB,RFC,RFD] = wl_getInterfaceIDs(nodes(1));

%Set up the interface for the experiment
wl_interfaceCmd(nodes,'RF_ALL','tx_gains',3,30);
wl_interfaceCmd(nodes,'RF_ALL','channel',2.4,11);

if(USE_AGC)
    wl_interfaceCmd(nodes,'RF_ALL','rx_gain_mode','automatic');
    wl_basebandCmd(nodes,'agc_target',-8);
    wl_basebandCmd(nodes,'agc_trig_delay', 511);
else
    wl_interfaceCmd(nodes,'RF_ALL','rx_gain_mode','manual');
    RxGainRF = 1; %Rx RF Gain in [1:3]
    RxGainBB = 4; %Rx Baseband Gain in [0:31]
    wl_interfaceCmd(nodes,'RF_ALL','rx_gains',RxGainRF,RxGainBB);
end

wl_interfaceCmd(nodes,'RF_ALL','tx_lpf_corn_freq',2); %Configure Tx for 36MHz of bandwidth
wl_interfaceCmd(nodes,'RF_ALL','rx_lpf_corn_freq',3); %Configure Rx for 36MHz of bandwidth

%We'll use the transmitter's I/Q buffer size to determine how long our
%transmission can be
txLength = nodes(1).baseband.txIQLen;

%Set up the baseband for the experiment
wl_basebandCmd(nodes,'tx_delay',0);
wl_basebandCmd(nodes,'tx_length',txLength);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Signal processing to generate transmit signal
% Here, we can send any signal we want out of each of the 8 transmit 
% antennas. For visualization, we'll send "pink" noise of 1MHz out of 
% each, but centered at different parts of the 40MHz band.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% First generate the preamble for AGC. The preamble corresponds to the
% short symbols from the 802.11a PHY standard
shortSymbol_freq = [0 0 0 0 0 0 0 0 1+i 0 0 0 -1+i 0 0 0 -1-i 0 0 0 1-i 0 0 0 -1-i 0 0 0 1-i 0 0 0 0 0 0 0 1-i 0 0 0 -1-i 0 0 0 1-i 0 0 0 -1-i 0 0 0 -1+i 0 0 0 1+i 0 0 0 0 0 0 0].';
shortSymbol_freq = [zeros(32,1);shortSymbol_freq;zeros(32,1)];
shortSymbol_time = ifft(fftshift(shortSymbol_freq));
shortSymbol_time = (shortSymbol_time(1:32).')./max(abs(shortSymbol_time));
shortsyms_rep = repmat(shortSymbol_time,1,30);

preamble_single = shortsyms_rep;
preamble_single = preamble_single(:);

shifts = floor(linspace(0,31,8));
for k = 1:8
   %Shift preamble for each antenna so we don't have accidental beamforming
   preamble(:,k) = circshift(preamble_single,shifts(k));
end

Ts = 1/(wl_basebandCmd(nodes(1),'tx_buff_clk_freq'));

payload = complex(randn(txLength-length(preamble),8),randn(txLength-length(preamble),8));
payload_freq = fftshift(fft(payload));
freqVec = linspace(-((1/Ts)/2e6),((1/Ts)/2e6),txLength-length(preamble));
BW = 1; %MHz 
noise_centerFreqs = linspace(-12,12,8);
for k = 1:8
    payload_freq((freqVec < (noise_centerFreqs(k) - BW/2)) | (freqVec > (noise_centerFreqs(k) + BW/2)),k)=0;
end
payload = ifft(fftshift(payload_freq));

txData = [preamble;payload];

node_tx1 = nodes(1);
node_tx2 = nodes(2);
node_rx = nodes(3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Transmit and receive signal using WARPLab
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

wl_basebandCmd(node_tx1,[RFA,RFB,RFC,RFD], 'write_IQ', txData(:,1:4)); %First 4 columns of txData is for primary tx
%wl_basebandCmd(node_tx2,[RFA,RFB,RFC,RFD], 'write_IQ', txData(:,5:8)); %Second 4 columns of txData is for secondary tx
wl_basebandCmd(node_tx2,[RFA,RFB], 'write_IQ', txData(:,5:6)); %Second 4 columns of txData is for secondary tx


wl_basebandCmd(node_tx1,'RF_ALL','tx_buff_en');
wl_basebandCmd(node_tx2,'RF_ALL','tx_buff_en');
wl_basebandCmd(node_rx,'RF_ALL','rx_buff_en');

wl_interfaceCmd(node_tx1,'RF_ALL','tx_en');
wl_interfaceCmd(node_tx2,'RF_ALL','tx_en');
wl_interfaceCmd(node_rx,'RF_ALL','rx_en');


set(gcf, 'KeyPressFcn','RUN_CONTINOUSLY=0;');
fprintf('Press any key to halt experiment\n')

while(1)
    eth_trig.send();

    rx_IQ = wl_basebandCmd(node_rx,[RFA,RFB],'read_IQ', 0, txLength);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Visualize results
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    t = [0:Ts:(txLength-1)*Ts].';
    figure(1);
    ax(1) = subplot(2,2,1);
    plot(t,real(rx_IQ(:,1)))
    title('Re\{rx\_IQ_{RFA}\}')
    xlabel('Time (s)')
    axis([0, max(t),-1,1])
    ax(2) = subplot(2,2,2);
    plot(t,real(rx_IQ(:,2)))
    title('Re\{rx\_IQ_{RFB}\}')
    xlabel('Time (s)')
    %linkaxes(ax,'x')
    axis([0, max(t),-1,1])

    FFTSIZE = 1024;

    ax(1) = subplot(2,2,3);
    rx_IQ_slice = rx_IQ(2049:end,1);
    rx_IQ_rs = reshape(rx_IQ_slice,FFTSIZE,length(rx_IQ_slice)/FFTSIZE);
    f = linspace(-20,20,FFTSIZE);
    fft_mag = abs(fftshift(fft(rx_IQ_rs)));
    plot(f,20*log10(mean(fft_mag,2)))
    title('FFT Magnitude of rx\_IQ_{RFA}')
    xlabel('Frequency (MHz)')
    axis([-20, 20,-20,40])
    
    ax(2) = subplot(2,2,4);
    rx_IQ_slice = rx_IQ(2049:end,2);
    rx_IQ_rs = reshape(rx_IQ_slice,FFTSIZE,length(rx_IQ_slice)/FFTSIZE);
    f = linspace(-20,20,FFTSIZE);
    fft_mag = abs(fftshift(fft(rx_IQ_rs)));
    plot(f,20*log10(mean(fft_mag,2)))
    title('FFT Magnitude of rx\_IQ_{RFB}')
    xlabel('Frequency (MHz)')
    %linkaxes(ax,'x')
    axis([-20, 20,-20,40])
    

    drawnow

    if (~RUN_CONTINOUSLY)
       break 
    end

end

wl_basebandCmd(nodes,'RF_ALL','tx_rx_buff_dis');
wl_interfaceCmd(nodes,'RF_ALL','tx_rx_dis');