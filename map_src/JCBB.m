
%-------------------------------------------------------
function H = JCBB (prediction, observations, compatibility, configuration)
% 
%-------------------------------------------------------
global Best;
% global configuration;

Best(1:observations.m) = zeros(observations.m, 1);

JCBB_R (prediction, observations, compatibility, [], 1);

H = Best';
% configuration.name = 'JOINT COMPATIBILITY B & B';


%-------------------------------------------------------
function JCBB_R (prediction, observations, compatibility, H, i)
% 
%-------------------------------------------------------
global Best;
%global configuration;

if i > observations.m % leaf node?
    if pairings(H) > pairings(Best(1:length(H))) % did better?
        Best(1:length(H)) = H;
    end
else
    individually_compatible = find(compatibility.ic(i,:));
    len = length(individually_compatible);
    for jj= 1:len
        j = individually_compatible(jj);
        if jointly_compatible(prediction, observations, [H j])
            JCBB_R(prediction, observations, compatibility, [H j], i + 1); %pairing (Ei, Fj) accepted 
        end
    end
%    if pairings(H) + length(compatibility.candidates.observations) - i >= pairings(Best) % can do better?
%    if pairings(H) + observations.m - i >= pairings(Best) % can do better?
    if pairings(H) + pairings(compatibility.AL(i+1:end)) >= pairings(Best(1:length(H))) % can do better?
        JCBB_R(prediction, observations, compatibility, [H; 0], i + 1); % star node: Ei not paired
    end
end


%-------------------------------------------------------
% 
%-------------------------------------------------------
function p = pairings(H)

p = length(find(H));



