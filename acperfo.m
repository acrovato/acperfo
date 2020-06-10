% Copyright 2020 Adrien Crovato
% 
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% 
%     http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.

%% acperf
% Aircraft performance computer
% Adrien Crovato
%
% Usage: acperfo(path/to/param_file)

function acperfo(fpath_)
% Clear
clearvars -except fpath_;
close all;
% Add to path
[fpath, fname, ~] = fileparts(fpath_);
addpath(fpath); % no need to addpath('private'); since this handled by MATLAB
% Call functions
results = struct();
[parameters, variables] = feval(fname);
results = straightlevel(parameters, variables, results);
results = climb(parameters, variables, results);
results = turn(parameters, variables, results);
results = glide(parameters, variables, results);
results = toldg(parameters, results);
% Remove from path
rmpath(fpath);
end
