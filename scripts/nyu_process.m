function nyu_process(datasetDir, sceneName, labeledDataset, outputDir)

disp(labeledDataset)

% The absolute directory of the 
sceneDir = sprintf('%s/%s', datasetDir, sceneName);

load(labeledDataset);

% Reads the list of frames.
frameList = get_synched_frames(sceneDir);

% Current frame number
frameCount = 0;

outputFileId = fopen([outputDir '/' sceneName '.txt'], 'wt+');

% Displays each pair of synchronized RGB and Depth frames.
for ii = 1 : numel(frameList)
  disp(sprintf('Processing frame %d out of %d', ii, numel(frameList)))
   
  frameCount = frameCount + 1;
    
  imgRgb = imread([sceneDir '/' frameList(ii).rawRgbFilename]);
  imgDepthRaw = swapbytes(imread([sceneDir '/' frameList(ii).rawDepthFilename]));
  
  [imgDepthProj, imgRgbProj] = project_depth_map(imgDepthRaw, imgRgb);
  imgScaledDepth = uint16(round(imgDepthProj * 1000));
  
  % TODO: The depth filling does not reproduce exactly the preprocessing
  %       from the semanticfusion
  imgDepthFilled = fill_depth_colorization(imgRgbProj, imgDepthProj);
  imgScaledDepthFilled = uint16(round(imgDepthFilled * 1000));
  
  accelData = get_accel_data([sceneDir '/' frameList(ii).accelFilename]);

  parts = regexp(frameList(ii).rawDepthFilename(3:end), '-', 'split');
  millis = str2double(parts{1});
  timestamp = int64(millis * 1000);

  imwrite(imgRgbProj, sprintf('%s/%s/%d_rgb.png', outputDir, sceneName, frameCount));
  imwrite(imgScaledDepth, sprintf('%s/%s/%d_depth.png', outputDir, sceneName, frameCount));                                                              
  imwrite(imgScaledDepthFilled, sprintf('%s/%s/%d_depthfilled.png', outputDir, sceneName, frameCount));                                                               
  
  % Checking the labelled entry
  labelFilename = 'None';
  compares = strcmp(rawRgbFilenames, [sceneName '/' frameList(ii).rawRgbFilename]);
  if any(compares)
      labelId = find(compares);
      labelFilename = sprintf('%s/%d_label.png', sceneName, frameCount);
      imwrite(uint16(labels(:,:,labelId)), sprintf('%s/%s', outputDir, labelFilename));
  end
  
  accelId = fopen(sprintf('%s/%s/%d_acceldata.txt', outputDir, sceneName, frameCount), 'wt+');
  fprintf(accelId, '%d %d %d %d\n', accelData(1), accelData(2), accelData(3), accelData(4));
  fclose(accelId);
  
  fprintf(outputFileId, '%d ./%s/%d_depth.png ./%s/%d_rgb.png %s %s %s\n', timestamp, ...
                                                                  sceneName, frameCount, ...
                                                                  sceneName, frameCount, ...
                                                                  frameList(ii).rawDepthFilename, ...
                                                                  frameList(ii).rawRgbFilename, ...
                                                                  labelFilename);

  
%% Visualization
  fig=figure(1);
  set(fig, 'Name', sprintf('Processing frame %d', ii), 'NumberTitle', 'off')

  subplot(1,4,1);
  imagesc(imgRgbProj);
  axis off;
  axis equal;
  title('Rgb');
  
  subplot(1,4,2);
  imagesc(imgScaledDepth);
  axis off;
  axis equal;
  title('Depth');
  
  subplot(1,4,3);
  imagesc(imgScaledDepthFilled);
  axis off;
  axis equal;
  title('Depth Filled');
  
  if any(strcmp(rawRgbFilenames, frameList(ii).rawRgbFilename))
    subplot(1,4,4);
    imagesc(labels(ii));
    axis off;
    axis equal;
    title(sprintf('Label at frame %d', ii));
  end
  
end

fclose(outputFileId);

% Generate label mapping
outputFileId = fopen([outputDir '/labelNames.txt'], 'wt+');

for ii = 1 : numel(names)
    fprintf(outputFileId, '%d %s\n', ii, names{ii});
end

fclose(outputFileId);

end
