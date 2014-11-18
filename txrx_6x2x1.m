function txrx_6x2x1(bs, user, eth_trig)

	USE_WARPLAB_TXRX = true;
	MODEL_NOISE = true;
	snr_dB = 20;

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



		wl_basebandCmd(bs.nodes(1),bs.txRadios{1}, 'write_IQ', bs.txData(:,1:4)); %First 4 columns of txData is for primary tx
	    wl_basebandCmd(bs.nodes(2),bs.txRadios{2}, 'write_IQ', bs.txData(:,5:end)); %Second 2 columns of txData is for secondary tx


	    wl_basebandCmd(bs.nodes(1),sum(bs.txRadios{1}),'tx_buff_en');
	    wl_basebandCmd(bs.nodes(2),sum(bs.txRadios{2}),'tx_buff_en');
	    wl_basebandCmd(bs.nodes(2),sum(bs.rxRadios{2}),'rx_buff_en');
	    wl_basebandCmd(user.nodes(1),sum(user.rxRadios{1}),'rx_buff_en');

	    wl_interfaceCmd(bs.nodes(1),sum(bs.txRadios{1}),'tx_en');
	    wl_interfaceCmd(bs.nodes(2),sum(bs.txRadios{2}),'tx_en');
	    wl_interfaceCmd(bs.nodes(2),sum(bs.rxRadios{2}),'rx_en');
	    wl_interfaceCmd(user.nodes(1),sum(user.rxRadios{1}),'rx_en');

	    eth_trig.send();

	    user.rx_IQ = wl_basebandCmd(user.nodes(1),user.rxRadios{1},'read_IQ', 0, length(bs.txData));
	    bs.rx_IQ = wl_basebandCmd(bs.nodes(2),bs.rxRadios{2},'read_IQ', 0, length(bs.txData));

	    user.agc_state = wl_basebandCmd(user.nodes(1),user.rxRadios{1},'agc_state');
	    bs.agc_state = wl_basebandCmd(bs.nodes(2),bs.rxRadios{2},'agc_state');

	    wl_basebandCmd([bs.nodes, user.nodes],'RF_ALL','tx_rx_buff_dis');
	    wl_interfaceCmd([bs.nodes, user.nodes],'RF_ALL','tx_rx_dis');

	end
 end