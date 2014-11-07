if USE_WARPLAB_TXRX == false; 


	if MODEL_NOISE == true
		% generate an AWGN noise vector
		rx_noise = (1/(sqrt(2)*db2mag(snr_dB))) *  complex(randn(numRxAntennas,txLength), ...
			                                         randn(numRxAntennas,txLength)) .';

			% generate an AWGN noise vector
		user_noise = (1/(sqrt(2)*db2mag(snr_dB))) *  complex(randn(numUsers,txLength), ...
			                                         randn(numUsers,txLength)) .';
	else
		rx_noise = zeros(numRxAntennas,txLength) .';

			% generate an AWGN noise vector
		user_noise = zeros(numUsers,txLength) .';
	end

	% apply the channel model. 
	bs_IQ = (H_selfInt * txData .') .' + rx_noise;
	bs_RSSI = mag2db(abs(bs_IQ));

	% apply the channel model. 
	user_IQ = (H_user * txData .') .' + user_noise;
	user_RSSI = mag2db(abs(user_IQ));


else

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Transmit and receive signal using WARPLab
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	%Set up the baseband for the experiment
	wl_basebandCmd(nodes,'tx_delay',0);
	wl_basebandCmd(nodes,'tx_length',txLength);


	wl_interfaceCmd(node_rx, RFA+RFB,'rx_en');
	wl_basebandCmd(node_rx, RFA+RFB,'rx_buff_en');


	if numTxAntennas == 2
		wl_basebandCmd(node_tx,[RFA, RFB], 'write_IQ', txData);
		wl_interfaceCmd(node_tx, RFA+RFB,'tx_en');
		wl_basebandCmd(node_tx, RFA+RFB,'tx_buff_en');
		
	
	elseif numTxAntennas == 4
		wl_basebandCmd(node_tx,[RFA, RFB, RFC, RFD], 'write_IQ', txData);
		wl_interfaceCmd(node_tx, 'RF_ALL','tx_en');
		wl_basebandCmd(node_tx, 'RF_ALL','tx_buff_en');
	else
		error('I cant work with this number of antennas')
	end
		

	eth_trig.send();
	rx_IQ = wl_basebandCmd(node_rx,[RFA, RFB],'read_IQ', 0, txLength);

	bs_IQ = rx_IQ(:,1);
	user_IQ = rx_IQ(:,2);

	bs_RSSI = mag2db(abs(bs_IQ));
	user_RSSI = mag2db(abs(user_IQ));
 

	wl_basebandCmd(nodes,'RF_ALL','tx_rx_buff_dis');
	wl_interfaceCmd(nodes,'RF_ALL','tx_rx_dis');
end




