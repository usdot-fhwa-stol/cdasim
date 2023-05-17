package example.com;

public class MyBridgeClass {
    public native int sum(int a, int b);
    public native int subtract(int a, int b);
    public native int dryTest();

    public static void main(String[] args) {
        MyBridgeClass myBridge = new MyBridgeClass();
        int result = myBridge.sum(3, 5);
        System.out.println("Java Sum: " + result);
        result = myBridge.subtract(88, 3);
        System.out.println("Java Subtract: " + result);
        result = myBridge.dryTest();
        System.out.println("Java Test: " + result);
    }

    static {
        System.loadLibrary("mybridge");
    }
}