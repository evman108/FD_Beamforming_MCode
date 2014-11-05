if USE_WARPLAB_TXRX == false; 


	% generate an AWGN noise vector
	rx_noise = (1/(sqrt(2)*db2mag(snr_dB))) *  complex(randn(numRxAntennas,txLength), ...
		                                         randn(numRxAntennas,txLength)) .';

	% apply the channel model. 
	rx_IQ = (H * txData .') .' + rx_noise;
	rx_RSSI = mag2db(abs(rx_IQ));


else

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Transmit and receive signal using WARPLab
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	%Set up the baseband for the experiment
	wl_basebandCmd(nodes,'tx_delay',0);
	wl_basebandCmd(nodes,'tx_length',txLength);

	wl_basebandCmd(node_tx,[RFA, RFB], 'write_IQ', txData);
	wl_interfaceCmd(node_tx, RFA+RFB,'tx_en');
	wl_interfaceCmd(node_rx, RFA,'rx_en');

	wl_basebandCmd(node_tx, RFA+RFB,'tx_buff_en');
	wl_basebandCmd(node_rx, RFA,'rx_buff_en');

	eth_trig.send();
	rx_IQ = wl_basebandCmd(node_rx,[RFA],'read_IQ', 0, txLength);
	rx_RSSI = mag2db(abs(rx_IQ));  % for now. We'll fix this later.   

	wl_basebandCmd(nodes,'RF_ALL','tx_rx_buff_dis');
	wl_interfaceCmd(nodes,'RF_ALL','tx_rx_dis');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Visualize results for channel estimation 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if plotWaveforms

	figure('Name', 'Received Signal Data Phase');
	colorOrder = get(gca, 'ColorOrder');
	for rxAnt = 1:numRxAntennas
		ax(rxAnt,1) = subplot(numRxAntennas+1,2,2*(rxAnt-1)+1);
		plot(0:(length(rx_IQ)-1),real(rx_IQ(:,rxAnt)), 'color', colorOrder(rxAnt,:))
		xlabel('Sample Index')
		title(strcat('Antenna #', num2str(rxAnt), ' Received I'))
		axis tight;
		ylim([-1.5,1.5]);

		ax(rxAnt,2) = subplot(numRxAntennas+1,2,2*(rxAnt-1)+2);
		plot(0:(length(rx_IQ)-1),imag(rx_IQ(:,rxAnt)), 'color', colorOrder(rxAnt,:))
		xlabel('Sample Index')
		title(strcat('Antenna #', num2str(rxAnt), ' Received Q'))
		axis tight
		ylim([-1.5,1.5])
	end

	linkaxes(ax,'x')
	subplot(numRxAntennas+1,1,numRxAntennas+1)
	hold on
	for rxAnt = 1:numRxAntennas
		plot(0:(length(rx_RSSI)-1),rx_RSSI(:,rxAnt), 'color', colorOrder(rxAnt,:))
	end
	hold off
	%legend('RFA','RFB','location','southeast')
	axis tight
	xlabel('Sample Index')
	title('Received RSSI')
end % plot plotWaveforms