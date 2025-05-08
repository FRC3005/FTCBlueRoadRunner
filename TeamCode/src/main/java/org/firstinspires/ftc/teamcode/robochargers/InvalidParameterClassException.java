package org.firstinspires.ftc.teamcode.robochargers;

/**
 * An exception class for situations where the various static parameter classes aren't structured in a way that the program needs.
 */
public class InvalidParameterClassException extends RuntimeException {
    /**
     * Creates a new {@code InvalidParameterClassException} without an error message.
     */
    public InvalidParameterClassException()
    {
    }

    /**
     * Creates a new {@code InvalidParameterClassException} with a user-specified error message.
     */
    public InvalidParameterClassException(String message)
    {
        super(message);
    }
}


