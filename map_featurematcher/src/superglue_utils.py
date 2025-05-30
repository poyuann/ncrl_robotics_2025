from pathlib import Path
import cv2
import matplotlib.cm as cm
import torch
import numpy as np


from superglue_lib.models.matching import Matching
from superglue_lib.models.utils import (AverageTimer, VideoStreamer,
                          make_matching_plot_fast, frame2tensor)

torch.set_grad_enabled(False)


def match_image(map_path): 
    """
    Wrapper function for matching two images, provides an interface to superglue model
    """
    center = None
    center_px = None
    image_height = None
    input = map_path
    # output_dir = "../results"
    output_dir = None
    image_glob = ['*.png', '*.jpg', '*.jpeg', '*.JPG']
    skip = 1
    max_length = 1000000


    # Important parameters to modify if you wish to improve the feature matching performance. 
    resize = [800] # Resize the image to this size before processing. Set to None to disable resizing.
    superglue = 'outdoor' # The SuperGlue model to use. Either 'indoor' or 'outdoor'.
    max_keypoints = 1000# -1 keep all keypoints  
    keypoint_threshold = 0.01 # Remove keypoints with low confidence. Set to -1 to keep all keypoints.
    nms_radius = 4 # Non-maxima suppression: keypoints with similar responses in a small neighborhood are removed.
    sinkhorn_iterations = 20 # Number of Sinkhorn iterations for matching.
    match_threshold = 0.5 # Remove matches with low confidence. Set to -1 to keep all matches.
    show_keypoints = True # Show the detected keypoints.
    no_display = True
    force_cpu = False # Force CPU mode. It is significantly slower, but allows the model to run on systems withou dedicated GPU.
    
   
    if len(resize) == 2 and resize[1] == -1:
        resize = resize[0:1]
    if len(resize) == 2:
        print('Will resize to {}x{} (WxH)'.format(
            resize[0], resize[1]))
    elif len(resize) == 1 and resize[0] > 0:
        print('Will resize max dimension to {}'.format(resize[0]))
    elif len(resize) == 1:
        print('Will not resize images')
    else:
        raise ValueError('Cannot specify more than two integers for --resize')

    
    device = 'cuda' if torch.cuda.is_available() and not force_cpu else 'cpu'
    print('Running inference on device \"{}\"'.format(device))
    config = {
        'superpoint': {
            'nms_radius': nms_radius,
            'keypoint_threshold': keypoint_threshold,
            'max_keypoints': max_keypoints
        },
        'superglue': {
            'weights': superglue,
            'sinkhorn_iterations': sinkhorn_iterations,
            'match_threshold': match_threshold,
        }
    }
    matching = Matching(config).eval().to(device)
    keys = ['keypoints', 'scores', 'descriptors']

    vs = VideoStreamer(input, resize, skip,
                       image_glob, max_length)
    frame, ret = vs.next_frame()
    assert ret, 'Error when reading the first frame (try different --input?)'

    frame_tensor = frame2tensor(frame, device)

    last_data = matching.superpoint({'image': frame_tensor})
    last_data = {k+'0': last_data[k] for k in keys}
    last_data['image0'] = frame_tensor
    last_frame = frame
    last_image_id = 0

    if output_dir is not None:
        print('==> Will write outputs to {}'.format(output_dir))
        Path(output_dir).mkdir(exist_ok=True)

    # Create a window to display the demo.
    if not no_display:
        cv2.namedWindow('SuperGlue matches', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('SuperGlue matches', 640*2*2, 480*2)
    else:
        print('Skipping visualization, will not show a GUI.')

    timer = AverageTimer()

    satellite_map_index = None # index of the satellite photo in the map where the best match was found
    index = 0 # index of the sattelite photo if a match was found
    max_matches = -1 # max number of matches, used to keep track of the best match
    MATCHED = False # flag to indicate if a match was found
    located_image = None # the image of the satellite photo where the best match was found
    features_mean = [0,0] # mean values of feature pixel coordinates 

    while True:
        
        #current sattelite image to be matched

        frame, ret = vs.next_frame()
        if not ret:
            # print('Finished demo_superglue.py')
            break
        timer.update('data')
        stem0, stem1 = last_image_id, vs.i - 1


        

        frame_tensor = frame2tensor(frame, device)
        pred = matching({**last_data, 'image1': frame_tensor})
        kpts0 = last_data['keypoints0'][0].cpu().numpy()
        kpts1 = pred['keypoints1'][0].cpu().numpy()
        matches = pred['matches0'][0].cpu().numpy()
        confidence = pred['matching_scores0'][0].detach().cpu().numpy()
        timer.update('forward')

        valid = matches > -1
        mkpts0 = kpts0[valid]
        mkpts1 = kpts1[matches[valid]]

        """
        Find image in sattelite map with findHomography        
        """
        #At least 4 matched features are needed to compute homography
        MATCHED = False
        print("Matches found:", len(mkpts1))
        if (len(mkpts1) >= 200): 
            perspective_tranform_error = False           
            M, mask = cv2.findHomography(mkpts0, mkpts1, cv2.RANSAC,5.0)
            h,w = last_frame.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            try: 
                dst = cv2.perspectiveTransform(pts,M)
            except:
                print("Perspective transform error. Abort matching.")
                perspective_tranform_error = True    

            if (len(mkpts1) > max_matches) and not perspective_tranform_error: 
                frame = cv2.polylines(frame,[np.int32(dst)],True,255,3, cv2.LINE_AA) 
                moments = cv2.moments(dst)
                cX = int(moments["m10"] / moments["m00"])
                cY = int(moments["m01"] / moments["m00"])
                center = (cX  ,cY) #shape[0] is Y coord, shape[1] is X coord
                #use ratio here instead of pixels because image is reshaped in superglue
                features_mean = np.mean(mkpts0, axis = 0)
                center_px = (int(features_mean[0]), int(features_mean[1])) #shape[0] is Y coord, shape[1] is X coord
                center_px = (features_mean[0], features_mean[1]) #shape[0] is Y coord, shape[1] is X coord
                image_height = (last_frame.shape[0], last_frame.shape[1])
                #Draw the center of the area which has been matched
                # cv2.circle(frame, center, radius = 10, color = (255, 0, 255), thickness = 5)
                # cv2.circle(last_frame, center_px, radius = 10, color = (255, 0, 0), thickness = 2)
                center = (cX / frame.shape[1] ,cY /frame.shape[0] )
                satellite_map_index = index
                max_matches = len(mkpts1)
                MATCHED = True
                    

        else:
            print("Photos were NOT matched")
      
        color = cm.jet(confidence[valid])
        k_thresh = matching.superpoint.config['keypoint_threshold']
        m_thresh = matching.superglue.config['match_threshold']
        small_text = [
            'Keypoint Threshold: {:.4f}'.format(k_thresh),
            'Match Threshold: {:.2f}'.format(m_thresh),
            'Image Pair: {:06}:{:06}'.format(stem0, stem1),
        ]
     
        out = make_matching_plot_fast(
            last_frame, frame, kpts0, kpts1, mkpts0, mkpts1, color, text='',
            path=None, show_keypoints=show_keypoints, small_text='')

        if MATCHED == True:
            located_image = out

        timer.update('viz')
        timer.print()  
        index += 1  

        if output_dir is not None:
            #stem = 'matches_{:06}_{:06}'.format(last_image_id, vs.i-1)
            stem = 'matches_{:06}_{:06}'.format(stem0, stem1)
            out_file = str(Path(output_dir, stem + '.png'))
            print('\nWriting image to {}'.format(out_file))
            cv2.imwrite(out_file, out)


    cv2.destroyAllWindows()
    vs.cleanup()
    
    return satellite_map_index, center, located_image, features_mean, last_frame, max_matches
    

