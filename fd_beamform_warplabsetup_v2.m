



	%Create a vector of node objects
	% Note: comment out wl_initNodes after running once to keep constant
	%  phase offsets among nodes sharing an RF reference clock.
	%  wl_initNodes() executes a reset of the MAX2829 transceivers, which
	%  forces a re-tune of the PLL, which changes the inter-node phases.
	nodes = wl_initNodes(NUMNODES);

	%Create a UDP broadcast trigger and primary node to be ready for it
	eth_trig = wl_trigger_eth_udp_broadcast;

	%TODO: Only trigger the transmitting node with an Ethernet trigger
	nodes.wl_triggerManagerCmd('add_ethernet_trigger',[eth_trig]);

	%Read Trigger IDs into workspace
	[T_IN_ETH,T_IN_ENERGY,T_IN_AGCDONE,T_IN_REG,T_IN_D0,T_IN_D1,T_IN_D2,T_IN_D3] =  wl_getTriggerInputIDs(nodes(1));
	[T_OUT_BASEBAND, T_OUT_AGC, T_OUT_D0, T_OUT_D1, T_OUT_D2, T_OUT_D3] = wl_getTriggerOutputIDs(nodes(1));

	%For the transmit node, we will allow Ethernet to trigger the buffer
	%baseband, the AGC, and debug0 (which is mapped to pin 8 on the debug
	%header)
	nodes(1).wl_triggerManagerCmd('output_config_input_selection',[T_OUT_BASEBAND,T_OUT_D0],[T_IN_ETH,T_IN_REG]);

	if(USE_EXTERNAL_TRIGGER)
	    %For the receive node, we will allow debug3 (mapped to pin 15 on the
	    %debug header) to trigger the buffer baseband, and the AGC
	    nodes(2).wl_triggerManagerCmd('output_config_input_selection',[T_OUT_BASEBAND,T_OUT_AGC],[T_IN_D3]);
	else
	    nodes(2).wl_triggerManagerCmd('output_config_input_selection',[T_OUT_BASEBAND,T_OUT_AGC],[T_IN_ETH,T_IN_REG]);
	end

	%For the receive node, we enable the debounce circuity on the debug 3 input
	%to deal with the fact that the signal may be noisy.
	nodes(2).wl_triggerManagerCmd('input_config_debounce_mode',[T_IN_D3],'enable'); 

	%Since the debounce circuitry is enabled, there will be a delay at the
	%receiver node for its input trigger. To better align the transmitter and
	%receiver, we can artifically delay the transmitters trigger outputs that
	%drive the buffer baseband and the AGC.
	nodes(1).wl_triggerManagerCmd('output_config_delay',[T_OUT_BASEBAND,T_OUT_AGC],[50]); %50ns delay

	%Get IDs for the interfaces on the boards. Since this example assumes each
	%board has the same interface capabilities, we only need to get the IDs
	%from one of the boards
	[RFA,RFB, RFC, RFD] = wl_getInterfaceIDs(nodes(1));


	%Set up the interface for the experiment
	wl_interfaceCmd(nodes,'RF_ALL','tx_gains',3,30);
	wl_interfaceCmd(nodes,'RF_ALL','channel',2.4,11);

	if(USE_AGC)
	    wl_interfaceCmd(nodes,'RF_ALL','rx_gain_mode','automatic');
	    wl_basebandCmd(nodes,'agc_target',-10);
	    wl_basebandCmd(nodes,'agc_trig_delay', 500);
	else
	    wl_interfaceCmd(nodes,'RF_ALL','rx_gain_mode','manual');
	    RxGainRF = 1; %Rx RF Gain in [1:3]
	    RxGainBB = 10; %Rx Baseband Gain in [0:31]
	    wl_interfaceCmd(nodes,'RF_ALL','rx_gains',RxGainRF,RxGainBB);
	end

	TX_NUM_SAMPS = nodes(1).baseband.txIQLen;
    % SAMP_FREQ = wl_basebandCmd(nodes(1),'tx_buff_clk_freq'); 


	%We'll use the transmitter's I/Q buffer size to determine how long our
	%transmission can be
	maxTxLength = nodes(2).baseband.txIQLen;
	sampFreq = wl_basebandCmd(nodes(2),'tx_buff_clk_freq'); 
	Ts = 1/sampFreq;

	node_tx = nodes(1);
	node_rx = nodes(2);