classdef fd_beamform_wl_control < hgsetget

	properties

		numTxAntennas = 0;
		numRxAntennas = 0;
		numUsers = 0;

		H_selfInt = [];
		H_user = [];


		USE_WARPLAB_TXRX = true;
		MODEL_NOISE = true;
		MODEL_DELAY = true;
		delay = 0;
		snr_dB = 100;


	end


	methods
		function self = fd_beamform_wl_control(numTxAntennas, numRxAntennas, numUsers)

			self.numTxAntennas = numTxAntennas;
			self.numRxAntennas = numRxAntennas;
			self.numUsers = numUsers;


			self.H_selfInt = zeros(numRxAntennas, numTxAntennas);

			self.H_user = zeros(numUsers, numTxAntennas);

		end	% end Constructor

		
		function generateChannelMatrices(self)

			self.H_selfInt = 1./sqrt(2) * (randn(self.numRxAntennas,self.numTxAntennas) ...
	                                  + j*randn(self.numRxAntennas,self.numTxAntennas)); 

			self.H_user = 1./sqrt(2) * (randn(self.numUsers,self.numTxAntennas) ...
                                   + j*randn(self.numUsers,self.numTxAntennas)); 
		end


		function txrx_6x2x1(self, bs, user, eth_trig)

			txLength = size(bs.txData,1);			

			if self.USE_WARPLAB_TXRX == false; 


				if self.MODEL_NOISE == true
					% generate an AWGN noise vector
					rx_noise = (1/(sqrt(2)*db2mag(self.snr_dB))) *  complex(randn(self.numRxAntennas,txLength), ...
						                                         randn(self.numRxAntennas,txLength)) .';

						% generate an AWGN noise vector
					user_noise = (1/(sqrt(2)*db2mag(self.snr_dB))) *  complex(randn(self.numUsers,txLength), ...
						                                         randn(self.numUsers,txLength)) .';
				else
					rx_noise = zeros(self.numRxAntennas,txLength) .';

						% generate an AWGN noise vector
					user_noise = zeros(self.numUsers,txLength) .';
				end

				if self.MODEL_DELAY
					txData_delayed = [zeros(self.delay, size(bs.txData,2)); ...
					                  bs.txData(1:end-self.delay,:)];
				else
					txData_delayed = bs.txData;
				end
				

				% apply the channel model. 
				bs.rx_IQ = (self.H_selfInt * txData_delayed .') .' + rx_noise;
				bs.RSSI = mag2db(abs(bs.rx_IQ ));

				% apply the channel model. 
				
			    user.rx_IQ = (self.H_user * txData_delayed .') .' + user_noise;
				user.RSSI = mag2db(abs(user.rx_IQ ));

				bs.rx_power_dBm   = bs.RSSI  ;
			    user.rx_power_dBm = user.RSSI ;


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

			    bs.RSSI = wl_basebandCmd(bs.nodes(2),bs.rxRadios{2},'read_RSSI', 0, length(bs.txData)/4);
			    user.RSSI = wl_basebandCmd(user.nodes(1), user.rxRadios{1}, 'read_RSSI', 0, length(bs.txData)/4)

			    user.agc_state = wl_basebandCmd(user.nodes(1),user.rxRadios{1},'agc_state');
			    bs.agc_state = wl_basebandCmd(bs.nodes(2),bs.rxRadios{2},'agc_state');

			    bs.rx_power_dBm = rssi2dBm(bs.RSSI, bs.agc_state(1), '24');
			    user.rx_power_dBm = rssi2dBm(user.RSSI, user.agc_state(1), '24');

			    wl_basebandCmd([bs.nodes, user.nodes],'RF_ALL','tx_rx_buff_dis');
			    wl_interfaceCmd([bs.nodes, user.nodes],'RF_ALL','tx_rx_dis');

			end %USE_WARPLAB_TXRX
		
		end %txrx_function

	end %methods

end %classdef


