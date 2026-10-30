"""Six-color candidates and temporal observation gates for the preliminary task.

All positions are image pixels. Scores are engineering quality measures, not
trained-model probabilities. A stable observation never grants motion authority.
"""
from collections import deque
import copy
import math
import time
import uuid
import cv2
import numpy as np
from ..qr_camera.task_codec import COLORS, parse_task

MATERIAL_DEVICE='/dev/v4l/by-id/usb-SYD_USB_Camera_200901010001-video-index0'
PALETTE={1:(40,40,215),2:(30,210,230),3:(200,65,45),4:(60,175,45),5:(35,35,35),6:(225,185,75)}


def default_profile():
    colors={}
    for color,bgr in PALETTE.items():
        lab=cv2.cvtColor(np.uint8([[bgr]]),cv2.COLOR_BGR2LAB)[0,0].astype(float)
        colors[str(color)]={'lab':lab.tolist(),'scale':[30.,22.,22.],
                            'calibrated':False,'sample_count':0,'v95':80. if color==5 else 255.}
    return {'version':1,'device':MATERIAL_DEVICE,'resolution':[1280,720],'processing_width':480,
            'plate_roi':None,'plate_roi_shape':'ellipse','grasp_roi':None,'colors':colors,'shape_reference':None,'shape_references':[],
            'min_area_px':300.,'max_area_fraction':.25,'min_score':.55,
            'max_frame_age_ms':250,'max_track_gap_ms':300,'stable_frames':5,
            'stable_span_ms':500,'stable_jitter_px':4.,'max_speed_px_s':8.,
            'request_timeout_ms':8000,'frame_margin_px':20,
            'component_color_dominance':.85,'component_min_support':.35,
            'metric_calibration':None}


def validate_profile(profile):
    if profile.get('version')!=1 or len(profile.get('resolution',[]))!=2:
        raise ValueError('配置版本或分辨率无效')
    width,height=profile['resolution']
    if not 160<=width<=4096 or not 120<=height<=2160:raise ValueError('分辨率范围无效')
    if not 320<=profile.get('processing_width',480)<=640:raise ValueError('处理图像宽度无效')
    if not 0<=profile.get('frame_margin_px',20)<=min(width,height)/4:raise ValueError('画面边缘余量无效')
    if not (.75<=profile.get('component_color_dominance',.85)<=.99 and
            .2<=profile.get('component_min_support',.35)<=.8):raise ValueError('物料轮廓颜色判定参数无效')
    if profile.get('plate_roi_shape','ellipse') not in ('ellipse','rectangle'):raise ValueError('物料区域形状无效')
    references=shape_references(profile)
    if len(references)>8:raise ValueError('外形参考最多8个')
    for reference in references:
        points=np.asarray(reference,dtype=float)
        if points.ndim!=2 or points.shape[1]!=2 or len(points)<3 or not np.isfinite(points).all():
            raise ValueError('外形参考轮廓无效')
    for name in ('plate_roi','grasp_roi'):
        roi=profile.get(name)
        if roi is not None and (len(roi)!=4 or not all(math.isfinite(float(v)) for v in roi) or
                               not (0<=roi[0]<roi[2]<=width and 0<=roi[1]<roi[3]<=height)):
            raise ValueError('区域坐标无效: '+name)
    for color in range(1,7):
        model=profile['colors'][str(color)]
        examples=model.get('examples',[model])
        if not isinstance(examples,list) or not 1<=len(examples)<=8:raise ValueError('颜色样本数量无效')
        for sample in [model]+examples:
            if len(sample['lab'])!=3 or len(sample['scale'])!=3 or not all(math.isfinite(float(v)) for v in sample['lab']+sample['scale']):
                raise ValueError('颜色采样值无效')
            if any(not 4<=v<=60 for v in sample['scale']):raise ValueError('颜色容差范围无效')
    if not (1<=profile['max_frame_age_ms']<=500 and 2<=profile['stable_frames']<=30 and
            100<=profile['stable_span_ms']<=3000 and 500<=profile['request_timeout_ms']<=12000):
        raise ValueError('帧龄、连续确认或等待期限无效')
    return profile


def sample_color(frame,rect,color):
    x1,y1,x2,y2=map(int,rect)
    height,width=frame.shape[:2]
    if color not in COLORS or not (0<=x1<x2<=width and 0<=y1<y2<=height):
        raise ValueError('颜色或采样框无效')
    patch=frame[y1:y2,x1:x2]
    if patch.size<64*3:raise ValueError('采样框太小，请框选物料的均匀表面')
    lab=cv2.cvtColor(patch,cv2.COLOR_BGR2LAB).reshape(-1,3).astype(float)
    hsv=cv2.cvtColor(patch,cv2.COLOR_BGR2HSV).reshape(-1,3)
    median=np.median(lab,axis=0)
    distance=np.linalg.norm(lab-median,axis=1)
    keep=distance<=np.quantile(distance,.8)
    lab,hsv=lab[keep],hsv[keep]
    median=np.median(lab,axis=0)
    spread=1.4826*np.median(np.abs(lab-median),axis=0)
    if np.max(spread)>25:raise ValueError('采样区域包含多种颜色，请缩小到物料表面')
    if color==5 and np.median(hsv[:,2])>130:raise ValueError('该区域不像黑色，请重新选择物料表面')
    if color!=5 and np.median(hsv[:,1])<25:raise ValueError('采样偏白或偏灰，请避开反光及背景')
    return {'lab':median.tolist(),'scale':np.clip(spread*2.5,8.,35.).tolist(),
            'calibrated':True,'sample_count':int(len(lab)),'v95':float(np.quantile(hsv[:,2],.95)),
            'sampled_at_unix':time.time()}


def contains(rect,bbox):
    if rect is None:return False
    x,y,w,h=bbox
    return rect[0]<=x and rect[1]<=y and x+w<=rect[2] and y+h<=rect[3]


def may_overlap(rect,bbox):
    """Unknown geometry remains relevant; touching a grasp boundary also blocks."""
    if rect is None or bbox is None:return True
    if len(bbox)!=4 or not all(math.isfinite(float(v)) for v in bbox):return True
    x,y,w,h=bbox
    if w<=0 or h<=0:return True
    return x<=rect[2] and x+w>=rect[0] and y<=rect[3] and y+h>=rect[1]


def shape_references(profile):
    refs=profile.get('shape_references',[])
    if refs:return refs
    legacy=profile.get('shape_reference')
    return [legacy] if legacy is not None else []


def append_shape_reference(profile,contour):
    points=np.asarray(contour,dtype=np.int32).reshape(-1,2)
    if len(points)<3 or cv2.contourArea(points.reshape(-1,1,2))<=0:raise ValueError('外形轮廓无效')
    refs=copy.deepcopy(shape_references(profile))
    if not any(cv2.matchShapes(np.int32(ref).reshape(-1,1,2),points.reshape(-1,1,2),cv2.CONTOURS_MATCH_I1,0)<.01 for ref in refs):
        refs.append(points.tolist())
    if len(refs)>8:refs=refs[:1]+refs[-7:]
    profile['shape_references']=refs;profile['shape_reference']=refs[0]
    return profile


def merge_color_sample(previous,current):
    """Keep labelled light/shadow examples; never widen a class into another color."""
    examples=copy.deepcopy(previous.get('examples',[previous])) if previous.get('calibrated') else []
    examples=[{k:v for k,v in item.items() if k!='examples'} for item in examples]
    item={k:v for k,v in current.items() if k!='examples'}
    # Repeated dragging of the same surface must not evict all other lighting samples.
    if not any(np.linalg.norm(np.float32(old['lab'])-np.float32(item['lab']))<3 for old in examples):
        examples.append(copy.deepcopy(item))
    if len(examples)>8:examples=examples[:1]+examples[-7:]
    result=copy.deepcopy(current);result['examples']=examples
    return result


def color_distance(channels, sample):
    """Same float32 LAB norm, using contiguous planes to avoid HWC temporaries."""
    center, scale = np.float32(sample['lab']), np.float32(sample['scale'])
    total = np.square((channels[0] - center[0]) / scale[0])
    for i in (1, 2):
        total += np.square((channels[i] - center[i]) / scale[i])
    return np.sqrt(total, out=total)


def _color_scores(channels, hsv, colors):
    """逐样本原位取最小值，避免先保留全部样本距离图再堆叠；判定公式不变。"""
    scores = []
    for color in range(1, 7):
        model = colors[str(color)]
        minimum = None
        for sample in model.get('examples', [model]):
            distance = color_distance(channels, sample)
            gate = (hsv[:, :, 2] <= min(145, max(70, sample['v95'] + 20))
                    if color == 5 else hsv[:, :, 1] >= 25)
            masked = np.where(gate, distance, 1e4)
            if minimum is None:
                minimum = masked
            else:
                np.minimum(minimum, masked, out=minimum)
        scores.append(minimum)
    return np.stack(scores)


class MaterialDetector:
    def __init__(self,profile):
        self.profile=validate_profile(copy.deepcopy(profile));self.rejected_regions=[];self.uncertain_regions=[]

    def detect(self,frame):
        self.rejected_regions=[];self.uncertain_regions=[]
        height,width=frame.shape[:2]
        if [width,height]!=self.profile['resolution']:return []
        ratio=min(1.,self.profile.get('processing_width',480)/width)
        small=cv2.resize(frame,None,fx=ratio,fy=ratio,interpolation=cv2.INTER_AREA) if ratio<1 else frame
        lab=cv2.cvtColor(small,cv2.COLOR_BGR2LAB).astype(np.float32)
        channels=np.ascontiguousarray(lab.transpose(2,0,1))
        hsv=cv2.cvtColor(small,cv2.COLOR_BGR2HSV)
        scores=_color_scores(channels,hsv,self.profile['colors'])
        winner=np.argmin(scores,axis=0)
        ranked=np.partition(scores,1,axis=0)
        best,runner=ranked[0],ranked[1]
        valid=(best<=3.) & (runner-best>=.35)
        roi=self.profile.get('plate_roi')
        roi_mask=np.full(small.shape[:2],255,np.uint8)
        if roi:
            x1,y1,x2,y2=[int(v*ratio) for v in roi]
            roi_mask[:]=0
            if self.profile.get('plate_roi_shape','ellipse')=='rectangle':
                cv2.rectangle(roi_mask,(x1,y1),(x2,y2),255,-1)
            else:
                cv2.ellipse(roi_mask,((x1+x2)//2,(y1+y2)//2),(max(1,(x2-x1)//2),max(1,(y2-y1)//2)),0,0,360,255,-1)
        interior=cv2.erode(roi_mask,np.ones((5,5),np.uint8))
        candidates=[]
        # Segment the continuous colored object before assigning its color. A blue
        # object's reflected cyan pixels must not become a second physical object.
        color_indices=[i for i in range(6) if i!=4 and self.profile['colors'][str(i+1)]['calibrated']]
        if not color_indices:color_indices=[0,1,2,3,5]
        foreground_supported=np.min(scores[color_indices],axis=0)<=3.
        object_mask=np.uint8((hsv[:,:,1]>=45) & (hsv[:,:,2]>=30) & foreground_supported)*255
        black_mask=np.uint8(valid & (winner==4))*255
        for is_black,mask in ((False,object_mask),(True,black_mask)):
            mask=cv2.bitwise_and(mask,roi_mask)
            mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
            mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
            contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area=cv2.contourArea(contour)/ratio**2
                if area<self.profile['min_area_px'] or area>width*height*self.profile['max_area_fraction']:continue
                x,y,w,h=cv2.boundingRect(contour)
                solidity=cv2.contourArea(contour)/max(1.,cv2.contourArea(cv2.convexHull(contour)))
                extent=cv2.contourArea(contour)/max(1,w*h)
                if not .22<=w/max(1,h)<=3.5 or solidity<.55 or extent<.25:continue
                moments=cv2.moments(contour)
                if moments['m00']<=0:continue
                center=[moments['m10']/moments['m00']/ratio,moments['m01']/moments['m00']/ratio]
                cmask=np.zeros(mask.shape,np.uint8);cv2.drawContours(cmask,[contour],-1,255,-1)
                inside=cmask>0;pixel_count=int(np.count_nonzero(inside))
                details={}
                if is_black:
                    color=5;distances=best[inside]
                else:
                    known=inside & valid & (winner!=4)
                    strong=known & (best<=4*(1-self.profile['min_score']))
                    counts=np.bincount(winner[strong],minlength=6);count=int(np.sum(counts))
                    dominant=int(np.argmax(counts));support=count/max(1,pixel_count)
                    fraction=float(counts[dominant]/max(1,count))
                    raw_counts=np.bincount(winner[known],minlength=6)
                    possible=[i+1 for i,v in enumerate(raw_counts) if v>=max(8,.10*max(1,int(np.sum(raw_counts))))]
                    bbox=[round(v/ratio,2) for v in (x,y,w,h)]
                    region={'bbox_px':bbox,'area_px':round(area,1),'possible_colors':possible,
                            'color_support':round(support,3),'color_dominance':round(fraction,3)}
                    if np.count_nonzero(known)<.25*pixel_count:
                        region['rejection_reason']='INSUFFICIENT_COLOR_EVIDENCE';self.rejected_regions.append(region);continue
                    if (support<self.profile.get('component_min_support',.35) or
                        fraction<self.profile.get('component_color_dominance',.85)):
                        region['rejection_reason']='AMBIGUOUS_OBJECT_COLOR';self.uncertain_regions.append(region);continue
                    color=dominant+1;distances=scores[dominant][strong & (winner==dominant)]
                    details={'color_support':round(support,3),'color_dominance':round(fraction,3),
                             'minority_color_fractions':{str(i+1):round(float(v/max(1,count)),3)
                                                        for i,v in enumerate(counts) if v and i!=dominant}}
                score=float(np.clip(1.-np.median(distances)/4.,0,1))
                original=(contour.astype(np.float32)/ratio).astype(np.int32)
                shapes=shape_references(self.profile)
                shape_distance=min(float(cv2.matchShapes(np.int32(shape).reshape(-1,1,2),original,
                    cv2.CONTOURS_MATCH_I1,0)) for shape in shapes) if shapes else None
                frame_clipped=x<=1 or y<=1 or x+w>=small.shape[1]-1 or y+h>=small.shape[0]-1
                roi_clipped=False
                if roi:
                    roi_clipped=bool(np.any(cmask[interior==0]>0))
                # Dark floor/shadow responses can pass the permissive pixel mask.
                # Keep weak color evidence as diagnostics, not extra material IDs.
                # A missing/weak real black target still leaves no valid target.
                if is_black and score<self.profile['min_score']:
                    self.rejected_regions.append({'bbox_px':[round(v/ratio,2) for v in (x,y,w,h)],
                        'area_px':round(area,1),'possible_colors':[5],'quality_score':round(score,3),
                        'shape_distance':shape_distance,'frame_clipped':frame_clipped,'roi_clipped':roi_clipped,
                        'rejection_reason':'INSUFFICIENT_BLACK_COLOR_EVIDENCE'})
                    continue
                if is_black and roi_clipped and shape_distance is not None and shape_distance>.2:
                    # A cut dark region with an unsupported shape is not a
                    # second identified material. Preserve the uncertainty so
                    # required black observations still cannot become ready.
                    self.uncertain_regions.append({'bbox_px':[round(v/ratio,2) for v in (x,y,w,h)],
                        'area_px':round(area,1),'possible_colors':[5],'quality_score':round(score,3),
                        'shape_distance':shape_distance,'frame_clipped':frame_clipped,'roi_clipped':roi_clipped,
                        'rejection_reason':'CLIPPED_BLACK_SHAPE_UNVERIFIED'})
                    continue
                candidates.append({'color_id':color,'color_name':COLORS[color],
                    'center_px':[round(v,2) for v in center],
                    'bbox_px':[round(v/ratio,2) for v in (x,y,w,h)],'area_px':round(area,1),
                    'quality_score':round(score,3),'color_calibrated':bool(self.profile['colors'][str(color)]['calibrated']),
                    'shape_distance':shape_distance,'shape_consistent':shape_distance is not None and shape_distance<=.2,
                    'shape_reference_count':len(shapes),
                    'frame_clipped':frame_clipped,'roi_clipped':roi_clipped,
                    'clipped':frame_clipped or roi_clipped,'contour':original,**details})
        return sorted(candidates,key=lambda c:c['area_px'],reverse=True)[:18]


class MaterialObserver:
    def __init__(self,profile,scope='all_remaining'):
        if scope not in ('all_remaining','local_target'):raise ValueError('观测范围无效')
        self.scope=scope
        self.profile=validate_profile(copy.deepcopy(profile));self.histories={}
        self.run_id=uuid.uuid4().hex;self.request_id=0;self.request_started=None
        self.last_sequence=None;self.last_capture=None;self.context=None;self.expired=False;self.request_succeeded=False

    def begin(self,now):
        self.request_id+=1;self.request_started=now;self.expired=False;self.request_succeeded=False;self.histories.clear()

    def reset_tracking(self):self.histories.clear()

    def process(self,candidates,sequence,captured,now,task=None,batch=1,index=0,resolution=None,uncertain_regions=None):
        p=self.profile;reasons=[]
        context=(task.get('raw') if task else None,task.get('accepted_at_unix') if task else None,batch,index)
        if context!=self.context:
            self.histories.clear();self.context=context
            # A changed target invalidates the old observation request.
            self.request_started=None;self.expired=False;self.request_succeeded=False
        age=(now-captured)*1000
        fresh=0<=age<=p['max_frame_age_ms']
        unique_frame=self.last_sequence is None or sequence>self.last_sequence
        capture_advances=self.last_capture is None or captured>self.last_capture
        frame_gap=self.last_capture is None or (captured-self.last_capture)*1000<=p['max_track_gap_ms']
        if not fresh or not unique_frame or not capture_advances or not frame_gap:
            self.histories.clear()
        if unique_frame and capture_advances:
            self.last_sequence=sequence;self.last_capture=captured
        if not fresh:reasons.append('CAMERA_STALE')
        if not unique_frame or not capture_advances:reasons.append('DUPLICATE_FRAME')
        if not frame_gap:reasons.append('FRAME_GAP')
        if resolution is not None and list(resolution)!=p['resolution']:reasons.append('RESOLUTION_MISMATCH')
        if not p['plate_roi']:reasons.append('PLATE_ROI_UNSET')
        if not p['grasp_roi']:reasons.append('GRASP_ROI_UNSET')
        parsed=None
        try:
            if task is not None:parsed=parse_task(task['raw'])
        except (ValueError,KeyError,TypeError):pass
        if parsed is None:reasons.append('TASK_INVALID')
        if batch not in (1,2) or index not in range(3):raise ValueError('批次或序位无效')
        target_color=parsed[f'batch{batch}'][index]['color_id'] if parsed else None
        batch_colors=[item['color_id'] for item in parsed[f'batch{batch}']] if parsed else []
        expected=set(batch_colors)
        required_remaining=set(batch_colors[index:])
        if self.scope=='local_target':
            expected={target_color} if target_color is not None else set()
            required_remaining=expected.copy()
        ambiguous_regions=[r for r in (uncertain_regions or []) if required_remaining & set(r.get('possible_colors',[]))]
        if self.scope=='local_target':
            ambiguous_regions=[r for r in ambiguous_regions if may_overlap(p['grasp_roi'],r.get('bbox_px'))]
        if ambiguous_regions:reasons.append('OBJECT_COLOR_AMBIGUOUS')
        matching_resolution=resolution is None or list(resolution)==p['resolution']
        if ambiguous_regions or not matching_resolution:self.histories.clear()
        groups={color:[c for c in candidates if c['color_id']==color] for color in range(1,7)}
        states={};stable_colors=set();moving_colors=set()
        if fresh and unique_frame and capture_advances and matching_resolution and not ambiguous_regions:
            for color,items in groups.items():
                if len(items)!=1:self.histories.pop(color,None);continue
                candidate=items[0]
                x,y,w,h=candidate['bbox_px'];edge_ok=(resolution is None or
                    min(x,y,resolution[0]-x-w,resolution[1]-y-h)>=p.get('frame_margin_px',20))
                if candidate['clipped'] or candidate['quality_score']<p['min_score'] or not edge_ok:
                    self.histories.pop(color,None);continue
                history=self.histories.setdefault(color,deque(maxlen=240))
                if history and (captured-history[-1][0])>p['max_track_gap_ms']/1000:
                    history.clear()
                history.append((captured,*candidate['center_px']))
                # Short bounded window: a target must remain stable, not average out an orbit.
                while (len(history)>p['stable_frames'] and
                       (captured-history[1][0])*1000>=p['stable_span_ms']):
                    history.popleft()
                span=history[-1][0]-history[0][0] if len(history)>1 else 0.
                coordinates=np.array([row[1:] for row in history],dtype=float)
                jitter=float(np.max(np.linalg.norm(coordinates-np.median(coordinates,axis=0),axis=1)))
                speeds=[math.dist(a[1:],b[1:])/(b[0]-a[0]) for a,b in zip(history,list(history)[1:]) if b[0]>a[0]]
                speed=float(max(speeds,default=0.))
                stable=(len(history)>=p['stable_frames'] and span*1000>=p['stable_span_ms'] and
                        jitter<=p['stable_jitter_px'] and speed<=p['max_speed_px_s'])
                if stable:stable_colors.add(color)
                if speed>p['max_speed_px_s']:moving_colors.add(color)
                states[color]={'samples':len(history),'span_ms':round(span*1000),
                               'jitter_px':round(jitter,2),'max_speed_px_s':round(speed,2),'stable':stable}
        visible_expected={color for color in expected if len(groups[color])==1}
        clipped_colors=[color for color in sorted(required_remaining) if len(groups[color])==1 and groups[color][0]['clipped']]
        low_quality_colors=[color for color in sorted(required_remaining) if len(groups[color])==1 and
                            groups[color][0]['quality_score']<p['min_score']]
        edge_colors=[];edge_distances={}
        if resolution is not None:
            width,height=resolution;margin=p.get('frame_margin_px',20)
            for color in sorted(required_remaining):
                if len(groups[color])!=1:continue
                x,y,w,h=groups[color][0]['bbox_px']
                distance=min(x,y,width-x-w,height-y-h);edge_distances[str(color)]=round(distance,2)
                if distance<margin:edge_colors.append(color)
        complete=bool(required_remaining) and required_remaining<=visible_expected and all(
            len(groups[color])<=1 for color in expected) and not (clipped_colors or low_quality_colors or edge_colors or ambiguous_regions)
        if clipped_colors:reasons.append('TASK_CANDIDATE_CLIPPED')
        if edge_colors:reasons.append('TASK_FRAME_MARGIN')
        if low_quality_colors:reasons.append('TASK_CANDIDATE_LOW_QUALITY')
        turntable='UNKNOWN'
        if expected & moving_colors:turntable='MOVING'
        elif (complete and visible_expected<=stable_colors and p['plate_roi'] and fresh and unique_frame and
              capture_advances and (resolution is None or list(resolution)==p['resolution'])):
            turntable='VISUALLY_STABLE'
        elif not complete:turntable='INCOMPLETE_VIEW'
        if turntable!='VISUALLY_STABLE':
            reasons.append(('LOCAL_VIEW_' if self.scope=='local_target' else 'TURNTABLE_')+turntable)
        targets=groups.get(target_color,[])
        target=targets[0] if len(targets)==1 else None
        if not targets:reasons.append('TARGET_NOT_FOUND')
        elif len(targets)>1:reasons.append('TARGET_AMBIGUOUS')
        if target:
            if not target['color_calibrated']:reasons.append('COLOR_UNCALIBRATED')
            if target['quality_score']<p['min_score']:reasons.append('LOW_QUALITY')
            if target['clipped']:reasons.append('TARGET_CLIPPED')
            if target_color not in stable_colors:reasons.append('TARGET_UNSTABLE')
            if not contains(p['grasp_roi'],target['bbox_px']):reasons.append('OUTSIDE_GRASP_ZONE')
            if target_color==5 and not target['shape_consistent']:reasons.append('BLACK_IDENTITY_UNVERIFIED')
        uncalibrated=[color for color in sorted(expected) if not p['colors'][str(color)]['calibrated']]
        if uncalibrated:reasons.append('TASK_COLORS_UNCALIBRATED')
        if self.request_started is None:reasons.append('REQUEST_NOT_STARTED')
        elapsed=0. if self.request_started is None else max(0.,now-self.request_started)*1000
        if self.request_started is not None and not self.request_succeeded and elapsed>=p['request_timeout_ms']:
            self.expired=True
        if not reasons and not self.expired:self.request_succeeded=True
        if self.expired:reasons.append('REQUEST_TIMEOUT')
        ready=not reasons
        public=[]
        for candidate in candidates:
            public.append({k:v for k,v in candidate.items() if k!='contour'})
        result={'schema_version':1,'run_id':self.run_id,'request_id':self.request_id,
                'frame_sequence':sequence,'frame_age_ms':round(age,2),'source_fresh':fresh,
                'captured_monotonic':captured,'observed_at_unix':time.time(),
                'task_raw':parsed['raw'] if parsed else None,'batch':batch,'order':index+1,'target_color':target_color,
                'required_remaining_colors':sorted(required_remaining),
                'clipped_task_colors':clipped_colors,'edge_task_colors':edge_colors,
                'task_frame_edge_distance_px':edge_distances,'uncertain_regions':uncertain_regions or [],
                'blocking_uncertain_regions':ambiguous_regions,
                'low_quality_task_colors':low_quality_colors,'frame_margin_px':p.get('frame_margin_px',20),
                'observation_scope':self.scope,'batch_colors':batch_colors,
                'turntable_state':'UNKNOWN' if self.scope=='local_target' else turntable,
                'local_view_state':turntable if self.scope=='local_target' else None,
                'tracks':states,'candidates':public,
                'target':None if target is None else {k:v for k,v in target.items() if k!='contour'},
                'uncalibrated_colors':uncalibrated,'observation_ready':ready if self.scope=='all_remaining' else False,
                'local_target_ready':ready if self.scope=='local_target' else False,
                'request_state':'IDLE' if self.request_started is None else ('TIMED_OUT' if self.expired else
                    ('CONFIRMED' if self.request_succeeded else 'WAITING')),
                'wait_elapsed_ms':round(elapsed),'request_timeout_ms':p['request_timeout_ms'],
                'reasons':list(dict.fromkeys(reasons)),'coordinate_unit':'pixel',
                'processing_width':p.get('processing_width',480),'metric_valid':False,
                'pose_mm':None,'action_allowed':False,'motion_enabled':False,'uart_opened':False}
        return result
