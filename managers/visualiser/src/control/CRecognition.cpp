#include "CRecognition.h"

#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))

CRecognition::CRecognition()
{
	colorArray = (unsigned char*)calloc(sizeof(unsigned char),COLOR_PRECISION*COLOR_PRECISION*COLOR_PRECISION);
	segmentArray = (SSegment*)calloc(sizeof(SSegment),MAX_SEGMENTS);
	stack = (int*)malloc(1024*768*sizeof(int));
	buffer = (int*)malloc(1024*768*sizeof(int));
	if (buffer == NULL)fprintf(stderr,"Not enough memory for buffer\n");
	int i = 0;
	memset(learned,0,sizeof(unsigned char)*3);
	memset(colorArray,0,COLOR_PRECISION*COLOR_PRECISION*COLOR_PRECISION);
	learned[0] = 10;
	learned[1] = 10;
	learned[2] = 60;
	tolerance = 30; 
	debug = false; 
	fprintf(stderr,"Recognition successfully initialized.\n");
}

CRecognition::~CRecognition()
{
	free(stack);
	free(buffer);
	free(segmentArray);
	free(colorArray);
}

//zvysi prah t, toleranci podobnosti vzorovemu pixelu
void CRecognition::increaseTolerance()
{
	tolerance+=5;
	if (tolerance > 400) tolerance = 400;
	fprintf(stdout,"Tolerance: %i\n",tolerance);
}

//snizi prah t, toleranci podobnosti vzorovemu pixelu
void CRecognition::decreaseTolerance()
{
	tolerance-=5;
	if (tolerance < 0) tolerance = 0;
	fprintf(stdout,"Tolerance: %i\n",tolerance);
}

//smaze indexovaci tabulku
void CRecognition::resetColorMap()
{
	memset(colorArray,0,COLOR_PRECISION*COLOR_PRECISION*COLOR_PRECISION);
}

void CRecognition::addPixel(unsigned char* a)
{
	//ulozi pixel do vzoroveho
	for (int i =0;i<3;i++) learned[i] = a[i];
	//prevede nauceny pixel do HSV
	rgbToHsv(learned[0],learned[1],learned[2],&learnedHue,&learnedSaturation,&learnedValue);

	//z daneho vzoru vytvori indexovaci tabulku
	unsigned char u[3];
	for (int r=0;r<256;r+=4){
		u[0] = r;
		for (int g=0;g<256;g+=4){
			u[1] = g;
			for (int b=0;b<256;b+=4){
				u[2] = b;
				int i = ((r/COLOR_STEP)*COLOR_PRECISION+g/COLOR_STEP)*COLOR_PRECISION+b/COLOR_STEP;
				if (evaluatePixel2(u) > 0) colorArray[i] = 1;
			}
		}
	}
	fprintf(stdout,"Learned RGB: %i %i %i, HSV: %i %i %i\n",learned[0],learned[1],learned[2],learnedHue,learnedSaturation,learnedValue);
}

//nauci se dany pixel
void CRecognition::learnPixel(unsigned char* a)
{
	//ulozi pixel do vzoroveho
	for (int i =0;i<3;i++) learned[i] = a[i];
	//prevede nauceny pixel do HSV
	rgbToHsv(learned[0],learned[1],learned[2],&learnedHue,&learnedSaturation,&learnedValue);

	//z daneho vzoru vytvori indexovaci tabulku
	unsigned char u[3];
	for (int r=0;r<256;r+=COLOR_STEP){
		u[0] = r;
		for (int g=0;g<256;g+=COLOR_STEP){
			u[1] = g;
			for (int b=0;b<256;b+=COLOR_STEP){
				u[2] = b;
				int i = ((r/COLOR_STEP)*COLOR_PRECISION+g/COLOR_STEP)*COLOR_PRECISION+b/COLOR_STEP;
				colorArray[i] = evaluatePixel3(u);
			}
		}
	}
	fprintf(stdout,"Learned RGB: %i %i %i, HSV: %i %i %i\n",learned[0],learned[1],learned[2],learnedHue,learnedSaturation,learnedValue);
}

//vrati podobnost pixelu metodou indexovaci tabulky - metoda z dilu IV
int CRecognition::evaluatePixelFast(unsigned char *a)
{
	int b = ((a[0]/COLOR_STEP)*COLOR_PRECISION+a[1]/COLOR_STEP)*COLOR_PRECISION+a[2]/COLOR_STEP;
	return colorArray[b];
}

//podobnost pixelu - prvni metoda z dilu III
float CRecognition::evaluatePixel1(unsigned char* a)
{
	float result = 1;
	for (int i =0;i<3;i++){ 	
		result += pow((int)a[i]-(int)learned[i],2);
	}
	return 1/result;
}

//podobnost pixelu - druha metoda z dilu III
float CRecognition::evaluatePixel2(unsigned char* a)
{
	float result = 0;
	for (int i =0;i<3;i++){ 	
		result += pow((int)a[i]-(int)learned[i],2);
	}
	result = sqrt(result);
	if (result > tolerance) result = 0; else result = 1;
	return result;
}

//podobnost pixelu - druha metoda z dilu III
float CRecognition::evaluatePixel3(unsigned char* a)
{
	float result = 0;
	unsigned int h;
	unsigned char s,v;
	rgbToHsv(a[0],a[1],a[2],&h,&s,&v);
	if (v > 10 && s > 10){
		result = result + pow((int)h-(int)learnedHue,2);
		result = result + pow((int)s-(int)learnedSaturation,2)/4;
		result = result + pow((int)v-(int)learnedValue,2)/16;
	}else{
		return 0;
	}
	result = sqrt(result);
	if (result > tolerance) result = 0; else result = 1;
	return result;
}

//hledani cesty - metoda z dilu V
SPixelPosition CRecognition::findPath(CRawImage* image)
{
	SPixelPosition result;
	result.x = 320;
	result.y = 240;
	//nejvyssi radek, ve kterem muze byt cesta viditelna
	int horizon = 1;
	//soucet stredu cesty v jednotlivych radcich
	int centerSum = 0;
	//aktualne prohledavany radek 
	int row = image->height-1;
	//minimalni sirka cesty
	int widthLimit = 20;
	//jak dlouha posloupnost pixelu nespravne barvy	musi byt nalezena, aby se ukoncilo prohledavani aktualniho radku 
	int noiseLimit = 20;
	int noise = 0;
	int width = 0;

	//stred cesty pro aktualni radek
	int center = image->width/2;
	//levy a pravy okraj cesty v aktualnim radku 
	int leftBorder,rightBorder;
	int pos = 0;
	do{
		//prohledavani radku smerem k levemu okraji
		leftBorder=center;	
		do{
			pos = 3*(leftBorder+row*image->width);
			leftBorder--;
			if (evaluatePixelFast(&image->data[pos])>0 && noise > 0)noise--;else noise++;
		}while (noise < noiseLimit && leftBorder > 0);

		//prohledavani radku smerem k pravemu okraji
		rightBorder=center;	
		do{
			pos = 3*(rightBorder+row*image->width);
			rightBorder++;
			if (evaluatePixelFast(&image->data[pos])>0 && noise > 0)noise--;else noise++;
		}while (noise < noiseLimit && rightBorder < image->width);
		//vypocet stredu a sirky cesty 
		width = rightBorder-leftBorder;
		center = (leftBorder+rightBorder)/2;
		centerSum+=center;

		//vykresleni vysledku do obrazku
		rightBorder = rightBorder - noiseLimit;	
		leftBorder = leftBorder + noiseLimit;	
		for (int i= 0;i<3;i++){
			pos = 3*(leftBorder+row*image->width+i);
			image->data[pos] = 255;image->data[pos+1] = 0;image->data[pos+2] = 0;
			pos = 3*(rightBorder+row*image->width-i);
			image->data[pos] = 255;image->data[pos+1] = 0;image->data[pos+2] = 0;
			pos = 3*(center+row*image->width-i);
			image->data[pos] = 0;image->data[pos+1] = 255;image->data[pos+2] = 0;
			pos = 3*(center+row*image->width+i);
			image->data[pos] = 0;image->data[pos+1] = 255;image->data[pos+2] = 0;
		}
		row--;
	}while (width > widthLimit && row > horizon);

	//vypocet rychlosti   
	result.x = centerSum/(image->height-row); 
	result.y = row;
	fprintf(stdout,"Middle %i %i\n",result.x,result.y); 
	return result;
}

//segmentace obrazu - metoda z dilu IV
SPixelPosition CRecognition::findSegment(CRawImage* image)
{
	CTimer timer;
	fprintf(stdout,"T0:%i\n",timer.getTime());timer.reset();timer.start();
	SPixelPosition result;
	result.x = 320;
	result.y = 240;

	int expand[4] = {image->width,-image->width,1,-1};
	int stackPosition = 0;

	int numSegments = 0;
	int len = image->width*image->height;

	fprintf(stdout,"T1:%i\n",timer.getTime());timer.reset();timer.start();
	//oznacime oblasti s hledanou barvou
	for (int i = 0;i<len;i++){
		 buffer[i] = -evaluatePixelFast(&image->data[3*i]);
	}

	fprintf(stdout,"T2:%i\n",timer.getTime());timer.reset();timer.start();
	//'ukrojime' okraje obrazu
	int pos =  (image->height-1)*image->width;
	for (int i = 0;i<image->width;i++){
		buffer[i] = 0;	
		buffer[pos+i] = 0;
	}
	for (int i = 0;i<image->height;i++){
		buffer[image->width*i] = 0;	
		buffer[image->width*i+image->width-1] = 0;
	}
	fprintf(stdout,"T3:%i\n",timer.getTime());timer.reset();timer.start();
	//zacneme prohledavani
	int position = 0; 
	for (int i = 0;i<len;i++){
		//pokud je nalezen pixel s hledanou barvou, 
		if (buffer[i] < 0 && numSegments < MAX_SEGMENTS){
			//fprintf(stdout,"T4:%i-",timer.getTime());timer.reset();timer.start();
			//zalozime dalsi segment
			buffer[i] = ++numSegments;
			segmentArray[numSegments-1].size = 1; 
			segmentArray[numSegments-1].x = i%image->width; 
			segmentArray[numSegments-1].y = i/image->width; 
			//a umistime souradnice pixelu na vrchol zasobniku
			stack[stackPosition++] = i;
			//dokud neni zasobnik prazdny
			while (stackPosition > 0){
				//vyjmeme ze zasobniku pozici posledne vlozeneho pixelu 
				position = stack[--stackPosition];
				//prohledame pixely na sousednich pozicich
				for (int j =0;j<4;j++){
					pos = position+expand[j];
					//a pokud maji hledanou barvu,
					if (buffer[pos] < 0){
						//vlozime je do zasobniku,
						stack[stackPosition++] = pos;
						//pridame jejich pozici do souradnic aktualniho segmentu
						segmentArray[numSegments-1].x += pos%image->width; 
						segmentArray[numSegments-1].y += pos/image->width; 
						//a zvetsime velikost segmentu 
						segmentArray[numSegments-1].size++; 
						buffer[pos] = numSegments;
					}
				}
			}
			//jakmile se zasobnik vyprazdni, tj. byly nalezeny vsechny pisely daneho segmentu  spocteme teziste segmentu
			segmentArray[numSegments-1].x = segmentArray[numSegments-1].x/segmentArray[numSegments-1].size; 
			segmentArray[numSegments-1].y = segmentArray[numSegments-1].y/segmentArray[numSegments-1].size; 
		}
	}

	fprintf(stdout,"T4:%i\n",timer.getTime());timer.reset();timer.start();
	//Najde nejvetsi segment
	int maxSize = 0;
	int index = 0;
	int i;
	for (i =0;i<numSegments;i++){
		if (maxSize < segmentArray[i].size){
			index = i;
			maxSize = segmentArray[i].size;
		}			
	}
	if (debug) fprintf(stdout,"Largest segment is %i %i %i %i\n",index,segmentArray[index].size,segmentArray[index].x,segmentArray[index].y);

	//a spocte jeho stred
	if (maxSize > 20){
		result.x = segmentArray[index].x;
		result.y = segmentArray[index].y;
	} 

	//vykreslime vysledek
	int j = 0;
	for (int i = 0;i<len&&false;i++){
		j = buffer[i];
		if (j > 0){
			image->data[i*3+j%3] = 0;
			image->data[i*3+(j+1)%3] = 255;
			image->data[i*3+(j+2)%3] = 255;
		}
	}
	fprintf(stdout,"T5:%i\n",timer.getTime());timer.reset();timer.start();
	return result;
}

//nalezeni teziste pixelu dane tridy - metoda z dilu III
SPixelPosition CRecognition::findMean(CRawImage* image)
{
	//priprava promennych pro vypocet
	SPixelPosition result;
	float sumX,sumY,eval,sumEval;
	sumX=sumY=eval=sumEval=0;
	int step = 1;
	int yconst = image->width;

	//vlastni vypocet
	for (int y = 0;y<image->height;y+=step){
		yconst = image->width*y;
		for (int x = 0;x<image->width;x+=step){
			//vyhodnoceni podobnosti pixelu
			eval=evaluatePixelFast(&image->data[3*(x+yconst)]);
			//vybarveni vysledku v obraze
			if (eval == 1){
				for (int i = 0;i<3;i++) image->data[3*(x+yconst)+i]=255-learned[i];
			}
			sumEval +=eval;
			sumX = sumX + x*eval;
			sumY = sumY + y*eval;
		}
	}
	//pokud byl nalezen alespon jeden pixel, je proveden vypocer teziste 
	if (sumEval > 0){
		sumX = sumX/sumEval;
		sumY = sumY/sumEval;
	}else{
		sumX = image->width/2;
		sumY = image->height/2;
	}
	//vypocet stredu 	
	result.x = (int)(sumX);
	result.y = (int)(sumY);
	return result;
}

//prevod RGB -> HSV, prevzato z www
void CRecognition::rgbToHsv(unsigned char r, unsigned char  g, unsigned char b, unsigned int *hue, unsigned char *saturation, unsigned char *value )
{
	float min, max, delta;
	float h,s,v;   

	h=s=v=0; 
	*saturation = (unsigned char) s;
	*value = (unsigned char) v;
	*hue = (unsigned int) h;

	min = min( r, min(g, b) );
	max = max( r, max(g, b) );
	v = max;			

	delta = max - min;

	if( max != 0 )
		s = min(delta*255 / max,255);	
	else {
		s = 0;
		h = -1;
		return;
	}

	if( r == max )
		h = ( g - b ) / delta;		// between yellow & magenta
	else if( g == max )
		h = 2 + ( b - r ) / delta;	// between cyan & yellow
	else
		h = 4 + ( r - g ) / delta;	// between magenta & cyan
	h = h*60;
	if (h<0) h+=360;
	*saturation = (unsigned char) s;
	*value = (unsigned char) v;
	*hue = (unsigned int) h;
}

