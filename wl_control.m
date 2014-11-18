classdef wl_control < hgsetget

	properties
		name = 'CommNode'
	
		txData
		rx_IQ
		nodes
		agc_state

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


