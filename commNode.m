classdef commNode < hgsetget

	properties
		name = 'CommNode'
	
		txFrame
		rx_IQ
		RSSI

		nodes
		agc_state
		rx_power_dBm

		RxGainRF
		RxGainBB

		txRadios
		rxRadios
	end

	properties (SetAccess = private)

	end

	methods
		function cn = commNode(nodes)

		cn.nodes = nodes;
		end	% end Constructor

		function printName(self)
			disp(self.name)
		end
	end



end


