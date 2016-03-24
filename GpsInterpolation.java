/**    
* @Title: GpsInterpolation.java 
* @Description: gps插值，主要使用三次样条插值算法 
* @author zwq   
* @Create date 2016-03-23
* @Modify 
*/ 
package com.carbox;
import java.util.Vector;

public class GpsInterpolation {
	
	final double EARTH_RADIUS = 6378137.0;  //地球半径
	final double ANGLE = Math.PI / 180.0; //角度  减少除法运算
	public Double[] X,Y;         //输入gps信息
	public Double[] H,A,B,C,D;   //计算中间数据
	public Double[] outX,outY;   //输出gps信息
	public int inSize;           //输入gps个数
	public int N;                //区间数   = inSize - 1
	public int outSize;          //输出gps个数
	boolean bTransposition;      //是否转置了坐标系
	boolean bChangeOrder;        //是否做了逆序操作  
	boolean bCubic;              //是否可以进行三次样条插值
	

	/**
	 * @Description          插值算法
	 * @param inPoint  in    gps数组
	 * @param outNum   in    输出数组个数
	 * @param pOutX    out   输出的经度数组
	 * @param pOutY    out   输出的纬度数组
	 * @return  是否成功
	 */
	public boolean GetInterpolationPts(Vector<String> inPoint,int outNum, Vector<Double> pOutX,Vector<Double> pOutY){
		inSize = 3;
		if(outNum <= inPoint.size() || inPoint.size() < inSize){
			return false;
		}
		   //输入为三个点
		bTransposition = false;
		bChangeOrder = false;
		N = inSize - 1;
		outSize = outNum;
		X = new Double[inSize];
		Y = new Double[inSize];
		
		for(int i = 0; i < inSize; i++){
			String[] tmp = inPoint.get(i).split(",");
			if(tmp.length != 2){
				return false;
			}
			try{
				X[i] = Double.valueOf(tmp[0]);
				Y[i] = Double.valueOf(tmp[1]);
			}catch(Exception e){
				return false;
			}
		}
		
		outX = new Double[outSize];
		outY = new Double[outSize];
		
		Double dDistance = GetGpsDistance(Y[0], X[0], Y[inSize - 1], X[inSize - 1]);
		Double dDirectionPre = GetGpsDirection(Y[0], X[0], Y[1], X[1]);
		Double dDirectionBack = GetGpsDirection(Y[1], X[1], Y[inSize - 1], X[inSize - 1]);
		Double dDirectionDiff = Math.abs(dDirectionPre - dDirectionBack); 
		if(dDistance < 10 || dDirectionDiff < 8 || (dDirectionDiff > 75 && dDirectionDiff < 285)){   //静止、运行缓慢、直线行驶 、近似直角点可以使用等分插值
			Bisection();
		}else{  
			bCubic = Transposition();
			if(bCubic){//满足条件 进行三次样条插值
				H = new Double[inSize];
				A = new Double[inSize];
				B = new Double[inSize];
				C = new Double[inSize];
				D = new Double[inSize];
				Spline();
				InterPolation();
			}else{ //不满足条件 进行等分插值
				Bisection();
			}
			
		}
		if(!bTransposition && !bChangeOrder){  //既不逆序也不转置
			for(int i = 0; i < outSize; i++){
				pOutX.add(outX[i]);
				pOutY.add(outY[i]);
			}
		}
		if(bTransposition && !bChangeOrder){  //转置但不逆序
			for(int i = 0; i < outSize; i++){
				pOutX.add(outY[i]);
				pOutY.add(outX[i]);
			}
		}
		if(!bTransposition && bChangeOrder){  //逆序但不转置
			for(int i = outSize - 1; i >= 0; i--){
				pOutX.add(outX[i]);
				pOutY.add(outY[i]);
			}
		}
		if(bTransposition && bChangeOrder){  //逆序且转置
			for(int i = outSize - 1; i >= 0; i--){
				pOutX.add(outY[i]);
				pOutY.add(outX[i]);
			}
		}
		return true;
	}
	
	
	/**
	 * @Description 转置逆序操作  判断是否需要转置以及逆序操作
	 * @return   是否满足三次样条插值条件
	 */
	private boolean Transposition(){
		boolean bRet = false;
		bTransposition = false;
		bChangeOrder = false;
		if(X[0] - X[1] <= 0 && X[1] - X[2] <= 0){//X轴递增
			bRet = true;
			return bRet;
		}else if(X[0] - X[1] >= 0 && X[1] - X[2] >= 0){//X轴递减 
			Double dTmp = X[2];  //调换第一个点和第三个点
			X[2] = X[0];
			X[0] = dTmp;
			dTmp = Y[2];
			Y[2] = Y[0];
			Y[0] = dTmp;
			bRet = true;
			bChangeOrder = true;
			return bRet;
		}
		if(Y[0] - Y[1] <= 0 && Y[1] - Y[2] <= 0){//Y轴递增
			bRet = true;
			bTransposition = true;
			Double dTmp;
			for(int i = 0; i < 3; i++){//转置
				dTmp = X[i];
				X[i] = Y[i];
				Y[i] = dTmp;
			}
			return bRet;
		}else if(Y[0] - Y[1] >= 0 && Y[1] - Y[2] >= 0){//Y轴递减  转置后再排序
			Double dTmp;
			for(int i = 0; i < 3; i++){
				dTmp = X[i];
				X[i] = Y[i];
				Y[i] = dTmp;
			}
			dTmp = X[2]; 
			X[2] = X[0];
			X[0] = dTmp;
			dTmp = Y[2];
			Y[2] = Y[0];
			Y[0] = dTmp;
			bRet = true;
			bTransposition = true;
			bChangeOrder = true;
			return bRet;
		}
		return bRet;
	}
	
	/**
	 * @Description 等分插值
	 * @return  无
	 */
	private void Bisection(){
		int half = outSize / 2;
		Double dStepPreX = (X[1] - X[0]) / half;
		Double dStepPreY = (Y[1] - Y[0]) / half;
		Double dStepBackX = (X[2] - X[1]) / half;
		Double dStepBackY = (Y[2] - Y[1]) / half;
		
		outX[0] = X[0];
		outY[0] = Y[0];
		outX[half] = X[1];
		outY[half] = Y[1];
		for(int i = 1; i < half; i++){
			outX[i] = outX[i - 1] + dStepPreX;
			outY[i] = outY[i - 1] + dStepPreY;
		}
		for(int i = half + 1; i < outSize; i++){
			outX[i] = outX[i - 1] + dStepBackX;
			outY[i] = outY[i - 1] + dStepBackY;
		}

		outX[outSize - 1] = X[2];
		outY[outSize - 1] = Y[2];
	}
	
	/**
	 * @Description  根据横坐标得到纵坐标的值
	 * @param dbInx  in  经度或者纬度
	 * @return 纬度或者经度
	 */
	
	public Double GetYbyX(Double dbInX){
		double E,E1,K,K1,H1; 
		Double dbOutY;
		int j ;    
		if(dbInX<X[0])   
		{   
			j = 0;   

		}   
		else if (dbInX > X[N])   
		{   
			j = N-1;   
		}   
		else  
		{   
			for (j=1;j<=N;j++)   
			{   
				if(dbInX<=X[j])   
				{   
					j=j-1;   

					break;   
				}   
			}   

		}   
		E=X[j+1]-dbInX;   
		E1=E*E;   
		K=dbInX-X[j];   
		K1=K*K;   
		H1=H[j]*H[j];   

		dbOutY=(3*E1-2*E1*E/H[j])*Y[j]+(3*K1-2*K1*K/H[j])*Y[j+1];   
		dbOutY=dbOutY+(H[j]*E1-E1*E)*B[j]-(H[j]*K1-K1*K)*B[j+1];   
		dbOutY=dbOutY/H1;
		return dbOutY;
	}
	
	/**
	 * @Description  计算输出gps数组
	 * @return 无
	 */
	
	private void InterPolation(){
		/*
		int dIndex = 1;
		if(bChangeOrder){
			dIndex = 2;
		}
		double dbStep = (X[dIndex] - X[dIndex - 1])/(outSize - 1);   
		for (int i = 0;i < outSize ;++i)   
		{   
			outX[i] = X[dIndex - 1] + dbStep*i;   
		}  
		for(int i=1;i<outSize;i++)   
		{   
			outY[i] = GetYbyX(outX[i]); 
		}   
		outY[0] = Y[dIndex - 1];  
		*/
		double dbStep = (X[N] - X[0])/(outSize - 1);   
		for (int i = 0;i < outSize ;++i)   
		{   
			outX[i] = X[0] + dbStep*i;   
		}  
		for(int i=1;i<outSize;i++)   
		{   
			outY[i] = GetYbyX(outX[i]); 
		}   
		outY[0] = Y[0]; 
	}
	
	/**
	 * @Description  三次样条算法
	 * @return 无
	 */
	private void Spline(){
		int i,P,L;   

		for (i=1;i<=N;i++)   
		{   
			H[i-1]=X[i]-X[i-1];   
		}   
		L=N-1;   
		for(i=1;i<=L;i++)   
		{   
			A[i]=H[i-1]/(H[i-1]+H[i]);   
			B[i]=3*((1-A[i])*(Y[i]-Y[i-1])/H[i-1]+A[i]*(Y[i+1]-Y[i])/H[i]);   
		}   
		A[0]=1.0;   
		A[N]=0.0;   
		B[0]=3*(Y[1]-Y[0])/H[0];   
		B[N]=3*(Y[N]-Y[N-1])/H[N-1];   

		for(i=0;i<=N;i++)   
		{   
			D[i]=2.0;   
		}   

		for(i=0;i<=N;i++)   
		{   
			C[i]=1-A[i];   
		}   

		P=N;   
		for(i=1;i<=P;i++)   
		{   

			if (  Math.abs(D[i]) <= 0.000001 )                                  
			{   
				return;   
			}   
			A[i-1]=A[i-1]/D[i-1];   
			B[i-1]=B[i-1]/D[i-1];   
			D[i]=A[i-1]*(-C[i])+D[i];   
			B[i]=-C[i]*B[i-1]+B[i];   
		}   
		B[P]=B[P]/D[P];   
		for(i=1;i<=P;i++)   
		{   
			B[P-i]=B[P-i]-A[P-i]*B[P-i+1];   
		} 
	}
	
	/**
	 * @Description 根据两点经纬度计算距离
	 * @param in 起点纬度
	 * @param in 起点经度
	 * @param in 终点纬度
	 * @param in 终点经度
	 * @return 计算出的距离
	 */
	private Double GetGpsDistance(double lat_a, double lng_a, double lat_b, double lng_b){
		if(lat_a == lat_b && lng_a == lng_b){
			return 0.0;
		}
		double radLat1 = (lat_a * ANGLE);
        double radLat2 = (lat_b * ANGLE);
        double a = radLat1 - radLat2;
        double b = (lng_a - lng_b) * ANGLE;
        double s = 2 * Math.asin(Math.sqrt(Math.pow(Math.sin(a / 2), 2)
                + Math.cos(radLat1) * Math.cos(radLat2)
                * Math.pow(Math.sin(b / 2), 2)));
        s = s * EARTH_RADIUS;
        s = Math.round(s * 10000) / 10000;
        return s;
	}
	
	/**
	 * @Description 计算航向角
	 * @param in 起点纬度
	 * @param in 起点经度
	 * @param in 终点纬度
	 * @param in 终点经度
	 * @return  航向角
	 */
	private Double GetGpsDirection(double latpre, double lngpre, double latbeh, double lngbeh){
		if(latpre == latbeh && lngpre == lngbeh){
			return 0.0;
		}
        double w1 = latpre * ANGLE;
        double j1 = lngpre * ANGLE;
        double w2 = latbeh * ANGLE;
        double j2 = lngbeh * ANGLE;
        double ret;

        ret = 4 * Math.pow(Math.sin((w1 - w2) / 2), 2) - Math.pow(Math.sin((j1 - j2) / 2) * (Math.cos(w1) - Math.cos(w2)), 2);
        ret = Math.sqrt(ret);
        double temp = (Math.sin(Math.abs(j1 - j2) / 2) * (Math.cos(w1) + Math.cos(w2)));
        ret = ret / temp;
        ret = Math.atan(ret) / Math.PI * 180;
        if (j1 > j2)
        {
            if (w1 > w2) ret += 180;
            else ret = 180 - ret;
        }
        else if (w1 > w2) ret = 360 - ret;
        return ret;
	}
	public static void main(String []args){
		GpsInterpolation demo = new GpsInterpolation();
		Vector<String> inPoint = new Vector<String>();
		inPoint.add("117.230044,31.821566");
		inPoint.add("117.230044,31.821566");
		inPoint.add("117.230044,31.821566");
		Vector<Double> outX = new Vector<Double>();
		Vector<Double> outY = new Vector<Double>();
		boolean bRet = demo.GetInterpolationPts(inPoint, 20, outX, outY);
		if(bRet){
			for(int i = 0; i < 20; i++){
				System.out.println(outX.get(i) + "," + outY.get(i));
			}
		}
	}
}
