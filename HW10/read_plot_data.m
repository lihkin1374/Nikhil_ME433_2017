function data = read_plot_data(mySerial)
  %nsamples = fscanf(mySerial,'%d');       % first get the number of samples being sent
  nsamples = 1000;
  data = zeros(nsamples,4);               % two values per sample:  maf, iir and fir
  times = zeros(nsamples,1);
  for i=1:nsamples
    %index(i,:) = fscanf(mySerial,'%d %d %d %d'); % read index
    %fprintf('Index number %d\n', index(i,:));
    data(i,:) = fscanf(mySerial,'%d %d %d %d'); % read in data from PIC32; assume ints, in mA
    fprintf('Index number %d %d %d %d\n', data(i,:));
    times(i) = (i-1)*0.01;                 % 0.01 s between samples
  end
  if nsamples > 1						        
    plot(times,data(:,2:4));            % plot the reference and actual
  else
    fprintf('Only 1 sample received\n');
    disp(data);
  end
  % print data
  title(sprintf('Filtered result'));
  ylabel('Data ');
  xlabel('Time (ms)');  
  legend('MAF','IIR', 'FIR');
end