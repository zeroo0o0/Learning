//
// Created by G枚ksu G眉vendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    //创建变量以储存直接和间接光照计算值
    Vector3f dir = { 0.0,0.0,0.0 };
    Vector3f indir = { 0.0,0.0,0.0 };
    //1.判断是否有交点：光线与场景中物体相交？
    Intersection inter = Scene::intersect(ray);
    //如果没交点
    if (!inter.happened) {
        return dir;//return 0,0,0
    }
    //2.ray打到光源了：说明渲染方程只用算前面的自发光项，因此直接返回材质的自发光项
    if (inter.m->hasEmission()) {
        if (depth == 0) {//第一次打到光
            return inter.m->getEmission();
        }
        else return dir;//弹射打到光，直接返回0，0.0
 
    }
    //3.ray打到物体：这个时候才开始进行伪代码后面的步骤
    
    //对场景中的光源进行采样，得到采样点light_pos和pdf_light
    Intersection light_pos;
    float pdf_light = 0.0f;
    sampleLight(light_pos, pdf_light);
   
    //3.1计算直接光照
 
    //物体的一些参数
    Vector3f p = inter.coords;
    Vector3f N = inter.normal.normalized();
    Vector3f wo = ray.direction;//物体指向场景
    //光源的一些参数
    Vector3f xx = light_pos.coords;
    Vector3f NN = light_pos.normal.normalized();
    Vector3f ws = (p - xx).normalized();//光源指向物体
    float dis = (p - xx).norm();//二者距离
    float dis2 = dotProduct((p - xx), (p - xx));
 
    //判断光源与物体间是否有遮挡：
    //发出一条射线，方向为ws 光源xx -> 物体p
    Ray light_to_obj(xx, ws);//Ray(orig,dir)
    Intersection light_to_scene = Scene::intersect(light_to_obj);
    //假如dis>light_to_scene.distance就说明有遮挡，那么反着给条件即可：
    if (light_to_scene.happened&& (light_to_scene.distance-dis>-EPSILON)) {//没有遮挡
        //为了更贴近伪代码，先设定一些参数
        Vector3f L_i = light_pos.emit;//光强
        Vector3f f_r = inter.m->eval(wo, -ws, N);//材质，课上说了，BRDF==材质，ws不参与计算
        float cos_theta = dotProduct(-ws, N);//物体夹角
        float cos_theta_l = dotProduct(ws, NN);//光源夹角
        dir = L_i * f_r * cos_theta * cos_theta_l / dis2 / pdf_light;
    }
 
    //3.2间接光照
    
    //俄罗斯轮盘赌
    //Scene.hpp中已经定义了P_RR:RussianRoulette=0.8
    float ksi = get_random_float();//随机取[0,1]
    if (ksi < RussianRoulette) {
        //计算间接光照
        
        //随机生成一个wi方向
        Vector3f wi = inter.m->sample(wo, N).normalized();//这里的wi其实没参与计算，返回的是一个随机的方向
        Ray r(p, wi);
        Intersection obj_to_scene = Scene::intersect(r);
        //击中了物体&&物体不是光源
        if (obj_to_scene.happened && !obj_to_scene.m->hasEmission()) {
            Vector3f f_r = inter.m->eval(wo, wi, N);//wo不参与计算
            float cos_theta = dotProduct(wi, N);
            float pdf_hemi = inter.m->pdf(wo, wi, N);
            indir = castRay(r, depth + 1) * f_r * cos_theta / pdf_hemi / RussianRoulette;
        }
    }
    return dir + indir;
}
