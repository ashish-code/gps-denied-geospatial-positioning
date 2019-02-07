%%
addpath ~/Projects/map_localization/code/objects/shared/

%%
paths = {'~/Projects/map_localization/data/odometry/libviso2_mono',
         '~/Projects/map_localization/data/odometry/libviso2_stereo'};
for j = 1:length(paths)
    cd(paths{j})
    d = dir('*.txt');
    for i = 1:length(d)
        [dirname,filename,ext] = fileparts(d(i).name);
        pose_to_observation (d(i).name, fullfile(dirname,[filename '.obs']));
    end
end

