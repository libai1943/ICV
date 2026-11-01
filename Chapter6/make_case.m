function task=make_case(number,source)
% Load the fixed ten-ICV illustrative task.
if nargin<1,number=1;end
if nargin<2,source=fullfile(fileparts(mfilename('fullpath')),'cases');end
task=jsondecode(fileread(fullfile(source,sprintf('case_%02d.json',number))));
end
