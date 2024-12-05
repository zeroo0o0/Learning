//
// Created by Göksu Güvendiren on 2019-05-14.
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
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
	Vector3f dir={0.0,0.0,0.0};
	Vector3f indir={0.0,0.0,0.0};
	Intersection inter=Scene::intersect(ray);
	if(!inter.happened) return dir;
	if(inter.m->hasEmission()){ //�򵽹�Դ 
		if(depth==0) return inter.m->getEmission();
		else return dir;
	}
	//Ȼ��ʼдα������Ķ���
	Intersection light_pos; 
	float pdf_light =0.0f;
	sampleLight(light_pos, pdf_light);
	//ֱ�ӹ���
	Vector3f p=inter.coords;
	Vector3f N=inter.normal.normalized();
	Vector3f wo=ray.direction;
	//��Դ 
	Vector3f x=light_pos.coords;
	Vector3f NN=light_pos.normal.normalized();
	Vector3f ws=(p-x).normalized();
	float dis=(p-x).norm();
	float dis2=dotProduct((p-x),(p-x));
	
	Ray light_to_obj(x,ws); //��Դ->���� 
	Intersection light_to_scene =Scene::intersect(light_to_obj);
	
    //����dis>light_to_scene.distance��˵�����ڵ�����ô���Ÿ��������ɣ�
	if(light_to_scene.happened &&(light_to_scene.distance-dis>-EPSILON)) //���ڵ� //EPSILON�ǿ��ǵ��˸�������������� 
	{
		Vector3f L_i=light_pos.emit;
		Vector3f f_r=inter.m->eval(wo,-ws,N); //BRDF==����
		float cos_theta =dotProduct(-ws,N);//�����巨�ߵļн�
		float cos_theta_l=dotProduct(ws,NN);//�͹�Դ���ߵļн� 
		dir=L_i*f_r*cos_theta*cos_theta_l/dis2/pdf_light;  //��ʽ 
	}
	
	//��ӹ��գ���Ҫ����˹���̶�
	float ksi=get_random_float(); //���ȡ[0,1]
	if(ksi<RussianRoulette)
	{
		Vector3f wi=inter.m->sample(wo,N).normalized();//�������һ��wi����
		Ray r(p,wi);
		Intersection obj_to_scene =Scene::intersect(r);
		//����������&&�Ҳ��ǹ�Դ 
		if(obj_to_scene.happened&&!obj_to_scene.m->hasEmission()){
			Vector3f f_r=inter.m->eval(wo,wi,N);
			float cos_theta=dotProduct(wi,N);
			float pdf_hemi =inter.m->pdf(wo,wi,N);
			
			indir=castRay(r,depth+1)*f_r*cos_theta/pdf_hemi/RussianRoulette;
		}
	}
	return dir+indir;
}
