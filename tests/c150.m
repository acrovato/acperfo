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

%% Cessna 150 data
% Test input file (Cessna 150)
% Adrien Crovato

function [p, v] = c150()
% User-defined (parameters)
p.mtow = 725; % kg, maximum takeoff weight
p.ar = 6.5; % aspect ratio
p.S = 15; % m^2, wing area

p.clmax = 1.2; % maximum lift coefficient
p.cd0 = 0.030; % zero-lift drag
p.eps = 0.75; % Oswald efficiency factor
p.nmax = 4.4; % maximum load factor

p.jet = false; % engine is jet (true) or propeller (false) based
p.dblade = 1.7; % m, probeller balde diameter (only when p.jet=false)
p.throttle = 100; % bhp or N, max power setting at sea level

% Variables (plot)
v.h_ = 12000; % ft, max altitude
v.v_ = 125; % kts, max velocity
v.ps_ = 10; % ft/s, max specific excess power
end
