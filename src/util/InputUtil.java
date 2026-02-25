package util;
import java.util.Scanner;
public class InputUtil {
    private Scanner scanner;
    public InputUtil() {
        scanner = new Scanner(System.in);
    }
    public int readInt(String prompt) {
        System.out.print(prompt);
        while (!scanner.hasNextInt()) {
            System.out.println("Please enter a valid integer.");
            scanner.next();
            System.out.print(prompt);
        }
        int value = scanner.nextInt();
        scanner.nextLine();
        return value;
    }
    public String readString(String prompt) {
        System.out.print(prompt);
        return scanner.nextLine().trim();
    }
    public void close() {
        scanner.close();
    }
}
