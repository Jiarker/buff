#include "Rune/RuneDetector.h"

bool RuneDetector::run(Mat_time frame)
{
    //cout<<endl;
    frame.copyTo(src);
    target = RuneArmor();
    preDeal(frame);
    if (findRuneArmor())
    {
        calCenter(target);
        setFoundState();
        std::vector<cv::Point2f> pts;
        target.getPoints(pts);
        last_target = target;
        return true;
    }
    //cout << "NO TARGET" << endl;
    if (lost_times < 100 && state != ArmorState::LOST)
    {
        state = ArmorState::FINDING;
        lost_times++;
        cout << "Lost Times: " << lost_times << endl;
    }
    else
    {
        lost_times = 0;
        state = ArmorState::LOST;
    }
    return false;
}

void RuneDetector::preDeal(Mat frame)
{
    Mat gray, gaussian;
    vector<Mat> bgr;

    // GaussianBlur(frame, frame, Size(7, 7), 1);
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    split(frame, bgr);

    Mat gray_bin, color_bin;
    if (enemy_color == COLOR::BLUE)
    {
        //cout<<"1"<<endl;
        threshold(gray, gray_bin, param.blue_brightness_thresh, 255, THRESH_BINARY);
        subtract(bgr[0], bgr[2], color_bin);
        threshold(color_bin, color_bin, param.blue_color_thresh, 255, THRESH_BINARY);
    }
    else
    {
        //cout<<"1"<<endl;
        threshold(gray, gray_bin, param.red_brightness_thresh, 255, THRESH_BINARY);
        subtract(bgr[2], bgr[0], color_bin);
        threshold(color_bin, color_bin, param.red_color_thresh, 255, THRESH_BINARY);
    }
    bin = gray_bin & color_bin;
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    //Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    dilate(bin, bin, element);
}

bool RuneDetector::findRuneArmor()
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    rune_armors.clear();
    //只检测外轮廓
    findContours(bin, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    //if (debug_param.debug_show_bin)
    {
        //cout<<"size of contours:"<<contours.size()<<endl;
        Mat_time temp;
        src.copyTo(temp);
        drawContours(temp, contours, -1, Scalar(255,0,0), 2, 8);
        // imshow("Binary", temp);
        // if (waitKey(1) == 'q')
        //     exit(0);
    }
    if(contours.size() == 0)
        return false;

    //通过面积筛选内外扇叶
    vector<vector<Point>>in_candidates;
    vector<vector<Point>>out_candidates;
    for(int i = 0; i < contours.size(); i++)
    {
        double contour_area = contourArea(contours[i]);
        //cout<<"i:"<<i<<"\tarea:"<<contour_area<<endl;
        if(contour_area > param.min_in_vane_area && contour_area < param.max_in_vane_area)
            in_candidates.push_back(contours[i]);
        else if(contour_area > param.min_out_vane_area && contour_area < param.max_out_vane_area)
            out_candidates.push_back(contours[i]);
    }

    //内扇叶筛选与比较
    Vane in_vane;
    int max_hull_num = 0;//同时判断in_vane是否已被赋值
    // cout<<"<in_candidates:"<<in_candidates.size()<<"\tout_candidates:"<<out_candidates.size()<<endl;
    // cout<<"in_hull_num:"<<in_candidates[0].hull_num<<endl;
    // cout<<"in_vane_contour_rrect_ratio:"<<in_candidates[0].contour_rrect_ratio<<endl;
    for(int i = 0; i < in_candidates.size(); i++)
    {
        
        //筛选
        //长宽
        //cout<<contourArea(in_candidates[i])<<endl;
        RotatedRect c_rrect = minAreaRect(in_candidates[i]);
        float long_side = max(c_rrect.size.width, c_rrect.size.height);
        float short_side = min(c_rrect.size.width, c_rrect.size.height);
        //cout<<long_side<<"   "<<short_side<<endl;
        float ls_ratio = long_side / short_side;
        //cout<<"ls_ratio:"<<ls_ratio<<endl;
        if(ls_ratio < param.min_in_vane_ls_ratio ||
           ls_ratio > param.max_in_vane_ls_ratio )
            continue;

        //cout<<"1.2"<<endl;
        float c_area = contourArea(in_candidates[i]);
        float contour_rrect_ratio = c_area / c_rrect.size.area();
        //cout<<"contour_rrect_ratio:"<<contour_rrect_ratio<<endl;
        if(contour_rrect_ratio < param.min_in_vane_contour_rrect_ratio ||
           contour_rrect_ratio > param.max_in_vane_contour_rrect_ratio)
           continue;
        //cout<<"1.3"<<endl;
        vector<Point>hull;
        approxPolyDP(in_candidates[i], hull, param.approx_epsilon*4, true);
        float hull_num = hull.size();
        //cout<<"hull_num:"<<hull_num<<endl;
        if(hull_num < param.min_in_vane_hull_num ||
           hull_num > param.max_in_vane_hull_num)
           continue;
        //比较(角点数)
        //cout<<"1.4"<<endl;
        if(hull_num > max_hull_num)
        {
            in_vane = Vane(c_rrect, long_side, short_side);
            max_hull_num = hull_num;
        }
    }
    if(max_hull_num == 0)
        return false;

    //外扇叶筛选与比较
    //cout<<endl;
    Vane out_vane;
    float min_distance = 999999.0;//同时判断out_vane是否已被赋值
    for(int i = 0; i < out_candidates.size(); i++)
    {
        //筛选
        // cout<<"i:"<<i
        //     <<"\tls_ratio:"<<out_candidates[i].ls_ratio
        //     <<"\tcontour_rrect_ratio:"<<out_candidates[i].contour_rrect_ratio
        //     <<"\thull_num:"<<out_candidates[i].hull_num
        //     <<endl;
        RotatedRect c_rrect = minAreaRect(out_candidates[i]);
        //长宽
        float long_side = max(c_rrect.size.width, c_rrect.size.height);
        float short_side = min(c_rrect.size.width, c_rrect.size.height);
        float ls_ratio = long_side / short_side;
        //cout<<"ls_ratio:"<<ls_ratio<<endl;
        if(ls_ratio < param.min_out_vane_ls_ratio ||
           ls_ratio > param.max_out_vane_ls_ratio )
            continue;

        float c_area = contourArea(out_candidates[i]);
        float contour_rrect_ratio = c_area / c_rrect.size.area();   
        //cout<<"contour_rrect_ratio:"<<contour_rrect_ratio<<endl;
        if(contour_rrect_ratio < param.min_out_vane_contour_rrect_ratio ||
           contour_rrect_ratio > param.max_out_vane_contour_rrect_ratio)
           continue;

        float distance = calDistance(in_vane.rrect.center, c_rrect.center);
        float distance_inlongside_ratio = distance / in_vane.long_side;
        //cout<<"i:"<<i<<"\tdistance_inlongside_ratio:"<<distance_inlongside_ratio<<endl;
        if(distance_inlongside_ratio < param.min_distance_inlongside_ratio ||
           distance_inlongside_ratio > param.max_distance_inlongside_ratio)
           continue;

        float derta_angle = fabs(in_vane.rrect.angle - c_rrect.angle);
        //cout<<"derta_angle:"<<derta_angle<<endl;
        //在x或y轴时角度会突变
        if(derta_angle > param.max_in_out_vane_derta_angle && 
           derta_angle < CV_PI / 2 - param.max_in_out_vane_derta_angle )
           continue;

        if(distance < min_distance)
        {
            out_vane = Vane(c_rrect, short_side, long_side);
            min_distance = distance;
        }
    }
    if(min_distance == 999999.0)
        return false;
    
    //cout<<"OK"<<endl;
    //模板匹配(待定)
    //moduleCmpare(in_vane,out_vane);
    target = RuneArmor(in_vane, out_vane, src.gyro_pose, param);
    return true;
}

bool RuneDetector::calCenter(RuneArmor &t_armor)
{
    cv::Point2f temp_circle_center = cv::Point2f(t_armor.circle_center);
    float R_roi = t_armor.out_vane.long_side;
    R_roi /= 3.5;

    //Rect2f(左上点x坐标，右上点y坐标，长，宽)
    cv::Rect2f R_roi_rect = cv::Rect2f(temp_circle_center.x - R_roi, temp_circle_center.y - R_roi, 2 * R_roi, 2 * R_roi);
    R_roi_rect &= cv::Rect2f(cv::Point2f(0, 0), cv::Point2f(bin.cols, bin.rows));
    //cout<<"R_roi_rect.width:"<<R_roi_rect.width<<"\tR_roi_rect.height:"<<R_roi_rect.height<<endl;
    if (R_roi_rect.width <= 10 || R_roi_rect.height <= 10)   
        return false;
    cv::Mat temp_R_roi = bin(R_roi_rect);

    if (debug_param.debug_show_bin)
    {
        cv::imshow("R_Bin", temp_R_roi);
        if (cv::waitKey(1) == 'q')  
            exit(0);
    }
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(temp_R_roi, contours, hierarchy, cv::RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, cv::Point2f(R_roi_rect.x, R_roi_rect.y));
    if (contours.size() <= 0)
        return false;

    //暂时不对最大距离进行判断
    //选出面积最大的轮廓
    int max_i = -1;
    double max_area = 0;
    for (int i = 0; i < contours.size(); i++)
    {
        double area = contourArea(contours[i]);
        if (area / t_armor.in_vane.rrect.size.area() < param.min_R_in_area_ratio)
            continue;
        if (area > max_area)
        {
            max_area = area;
            max_i = i;
        }
    }
    if (max_i < 0)    
        return false;

    //判断
    cv::RotatedRect R_rrect = minAreaRect(contours[max_i]);
    if (calDistance(t_armor.circle_center, R_rrect.center) / R_roi > param.max_R_to_predict_distance_ratio)
        return false;

    t_armor.circle_center = R_rrect.center;
    return true;
}

bool RuneDetector::ifOldArmor()
{
    if (last_target.in_vane.rrect.boundingRect().empty())   
        return false;
        float max_diff_distance_ratio= calDistance(target.out_vane.rrect.center, last_target.out_vane.rrect.center) / target.out_vane.long_side;
        //cout<<"max_diff_distance_ratio:"<<max_diff_distance_ratio<<endl;
    if (calDistance(target.out_vane.rrect.center, last_target.out_vane.rrect.center) / target.out_vane.long_side > param.max_diff_distance_ratio)
        return false;
    return true;
}


RuneDetector::RuneDetector()
{
    int color;
    cv::FileStorage fs("../Configure/Settings.xml", cv::FileStorage::READ);
    fs["enemy_color"] >> color;
    fs.release();
    setEnemyColor((COLOR)color);
}