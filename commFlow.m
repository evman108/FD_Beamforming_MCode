classdef commFlow < hgsetget

	properties
		name = 'Flow'
		
		preamble
		
		tx_data
		tx_syms
		tx_syms_mat
		tx_payload_mat
		ifft_in_mat

		tx_payload_vec
		tx_vec
		tx_vec_padded
		tx_vec_air

		selfInt_ifft_in_mat
		tx_selfInt_vec
		tx_selfInt_vec_air

		sig_delay = 0;
		selfint_delay = 0;

		rx_vec_air
		raw_rx_dec
		raw_rx_dec_cx
		rx_H_est
		rx_selfInt_H_est
		rx_cfo_est_lts
		payload_syms_mat
		rx_syms
		rx_data
		rx_H_est_plot

		selfint_payload_ind_no_delay
		selfint_payload_ind
		selfint_lts_payload_offset
		selfint_lts_ind

		payload_ind_no_delay
		payload_ind
		lts_ind
		lts_second_peak_index
		lts_peaks
		LTS1
		LTS2
		lts_payload_offset = 0

		


		pilots_mat


		sym_errs = 0;
		bit_errs = 0;
		rx_evm = 0;

		node_tx
		node_rx



	end

	properties (SetAccess = private)

	end

	methods
		function cf = commFlow()  

		end	% end Constructor

		function printName(self)
			disp(self.name)
		end
	end



end


