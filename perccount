function  perccount(jj,maxjj)
%Reports the percentage of the job done to the screen.
%
%  PERCOUNT(I,Imax)
%    I is the current iteration between 1 and Imax
%    Imax is the maximum number of iterations
%
%  Do not print anything to the screen between calls of this function!
%

%  title - s cmspike/perccount  vr - 1.2 
%  author - bodc/alead date - 2006may16

%  Updated 2009May14
%    At suggestion of John D'Errico renamed internal variable "max" to 
%    "maxjj"
%    Also following D'Errico's suggestions the following functionality has
%    been added:
%      1. An invocation check - checks that two input arguments are
%      supplied

  persistent lastCall;
  if(nargin  ==  2)
    if(lastCall  ~=  floor(((jj-1)/maxjj) * 100))
      if(jj  ~=  1)
        fprintf(1,'\b\b\b');
      else
        fprintf(1,'\n\tPercentage complete: ');
      end
      pc_done  =  num2str(floor(((jj-1)/maxjj) * 100));
      if(length(pc_done)  ==  1)
        pc_done(2)  =  pc_done(1);
        pc_done(1)  =  '0';
      end
      fprintf(1,'%s%%',pc_done);
    end
    lastCall  =  floor(((jj-1)/maxjj) * 100);
    if(jj  ==  maxjj)
      fprintf(1,'\n\n');
    end
  else
    error('Error: PERCCOUNT needs two input arguments...');
  end