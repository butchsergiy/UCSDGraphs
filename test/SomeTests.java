/**
 * Created by BSV on 29.09.2016.
 */
public class SomeTests {

    private int a = 10;

    public SomeTests(int a) {
        this.a = a;
    }

    public void printI(){
        System.out.printf("\n var a = %d \n", a);
    }


    public static void main(String ... args){

        SomeTests ob = new SomeTests(1);
        ob.printI();

    }


}
