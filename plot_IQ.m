function handle = plot_IQ(rx_IQ, rx_RSSI, numRxAntennas, name)

	h = figure('Name', name);
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

end