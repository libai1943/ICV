function task=make_case(number,source)
% Convert author-supplied boundary values and obstacles; no legacy solver code.
if nargin<2,source=fullfile(fileparts(mfilename('fullpath')),'cases');end
task=jsondecode(fileread(fullfile(source,sprintf('case_%02d.json',number))));
end
