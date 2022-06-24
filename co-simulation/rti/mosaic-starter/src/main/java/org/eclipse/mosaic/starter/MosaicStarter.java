/*
 * Copyright (c) 2020 Fraunhofer FOKUS and others. All rights reserved.
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contact: mosaic@fokus.fraunhofer.de
 */

package org.eclipse.mosaic.starter;

import org.eclipse.mosaic.lib.util.cli.CommandLineParser;
import org.eclipse.mosaic.lib.util.objects.ObjectInstantiation;
import org.eclipse.mosaic.rti.MosaicComponentProvider;
import org.eclipse.mosaic.rti.api.MosaicVersion;
import org.eclipse.mosaic.rti.config.CHosts;
import org.eclipse.mosaic.starter.cli.MosaicParameters;
import org.eclipse.mosaic.starter.config.CRuntime;
import org.eclipse.mosaic.starter.config.CScenario;

import ch.qos.logback.classic.LoggerContext;
import org.apache.commons.cli.ParseException;
import org.apache.commons.lang3.StringUtils;
import org.apache.commons.lang3.SystemUtils;
import org.apache.commons.lang3.Validate;
import org.apache.commons.lang3.exception.ExceptionUtils;
import org.apache.commons.lang3.time.DurationFormatUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.awt.Desktop;
import java.io.ByteArrayOutputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * This class contains a main method that configures a Eclipse MOSAIC simulation based
 * on an XML file.
 */
public class MosaicStarter {

    private static final Path RUNTIME_CONFIG = Paths.get("etc", "runtime.json");

    private static final Path LOGBACK_CONFIG = Paths.get("etc", "logback.xml");

    private static final Path HOSTS_CONFIG = Paths.get("etc", "hosts.json");

    private static final Path VISUALIZER_PATH = Paths.get("tools", "web", "visualizer.html");

    private static final Path LIBRARIES = Paths.get("lib");

    /**
     * Reads all information from the given XML file and starts Eclipse MOSAIC with the
     * read parameters. If successful, the process exits with status code {@code 0},
     * in the case of an error with status code {@code -1}.
     *
     * @param args parameter list from the command line interface
     */
    public static void main(String[] args) {
        try {
            new MosaicStarter().execute(args);
            System.exit(0);
        } catch (ExecutionException e) {
            System.exit(-1);
        }
    }

    /**
     * Reads all information from the given XML file and starts Eclipse MOSAIC with the
     * read parameters.
     *
     * @param arguments parameter list from the command line interface
     * @throws ExecutionException whenever the execution is configured incorrectly
     */
    public void execute(String[] arguments) throws ExecutionException {
        if (arguments.length == 0) {
            printVersionAndCopyrightInfo();
        }

        final MosaicParameters params = readParametersFromCli(arguments);

        final ClassLoader classLoader = createClassLoader();

        final Path scenarioConfigurationFile = findScenarioConfigurationFile(params);
        final Path scenarioDirectory = extractScenarioDirectory(scenarioConfigurationFile);

        final Path runtimeConfigurationFile = params.runtimeConfiguration != null ? Paths.get(params.runtimeConfiguration) : RUNTIME_CONFIG;
        final CRuntime runtimeConfiguration = loadRuntimeConfiguration(runtimeConfigurationFile);

        final Path hostConfigurationFile = params.hostsConfiguration != null ? Paths.get(params.hostsConfiguration) : HOSTS_CONFIG;
        final CHosts hostsConfiguration = loadHostsConfiguration(hostConfigurationFile);

        final MosaicSimulation simulation = createSimulation()
                .setRuntimeConfiguration(runtimeConfiguration)
                .setHostsConfiguration(hostsConfiguration)
                .setRealtimeBrake(StringUtils.isNumeric(params.realtimeBrake) ? Integer.parseInt(params.realtimeBrake) : 0)
                .setLogbackConfigurationFile(params.loggerConfiguration != null ? Paths.get(params.loggerConfiguration) : LOGBACK_CONFIG)
                .setLogLevelOverride(params.logLevel)
                .setComponentProviderFactory(createComponentProviderFactory(runtimeConfiguration, classLoader))
                .setClassLoader(classLoader);

        if (StringUtils.isNotEmpty(params.watchdogInterval) && StringUtils.isNumeric(params.watchdogInterval)) {
            simulation.setWatchdogInterval(Integer.parseInt(params.watchdogInterval));
        }
        if (StringUtils.isNotEmpty(params.externalWatchDog) && StringUtils.isNumeric(params.externalWatchDog)) {
            simulation.setExternalWatchdogPort(Integer.parseInt(params.externalWatchDog));
        }

        final CScenario scenarioConfiguration = loadJsonConfiguration(scenarioConfigurationFile, CScenario.class);

        if (params.randomSeed != null) {
            scenarioConfiguration.simulation.randomSeed = params.randomSeed;
        }

        if (params.startVisualizer) {
            startVisualizerInBrowser();
        }

        final MosaicSimulation.SimulationResult result = simulation.runSimulation(scenarioDirectory, scenarioConfiguration);

        if (result.success) {
            printSuccessfulSimulation(simulation.getLogger(), result.durationMs);
        } else {
            printErrorInformation(simulation.getLogger(), result.exception);
            throw new ExecutionException();
        }
    }

    protected MosaicSimulation createSimulation() {
        return new MosaicSimulation();
    }

    protected void printVersionAndCopyrightInfo() {
        System.out.println("Eclipse MOSAIC [Version " + MosaicVersion.get().toString() + "]");
        System.out.println("Copyright (c) 2021 Fraunhofer FOKUS and others. All rights reserved.");
        System.out.println("License EPL-2.0: Eclipse Public License Version 2 [https://eclipse.org/legal/epl-v20.html].");
        System.out.println();
    }

    protected MosaicParameters readParametersFromCli(String[] args) throws ExecutionException {
        final CommandLineParser<MosaicParameters> cli = new CommandLineParser<>(MosaicParameters.class)
                .usageHint("mosaic -c <CONFIG-FILE> \n       mosaic -s <SCENARIO> \n\n", null, null);
        try {
            final MosaicParameters params = cli.parseArguments(args, new MosaicParameters());
            if (params == null) {
                throw new ExecutionException();
            }

            Validate.isTrue(params.configurationPath != null || params.scenarioName != null,
                    "Either the configuration path (-c) or the name of the scenario (-s) must be given.");
            return params;
        } catch (IllegalArgumentException | ParseException e) {
            printUsage(cli, e.getLocalizedMessage());
            throw new ExecutionException();
        }
    }

    protected Path findScenarioConfigurationFile(MosaicParameters params) throws ExecutionException {
        Path configurationPath = params.configurationPath != null
                ? Paths.get(params.configurationPath)
                : Paths.get("scenarios", params.scenarioName, "scenario_config.json");

        if (!Files.exists(configurationPath)) {
            printAndLog("Could not find configuration file in path " + configurationPath);
            throw new ExecutionException();
        }
        if (!Files.isReadable(configurationPath)) {
            printAndLog("Could not access configuration file in path " + configurationPath);
            throw new ExecutionException();
        }
        return configurationPath;
    }

    private Path extractScenarioDirectory(Path scenarioConfigurationFile) throws ExecutionException {
        Path parent = scenarioConfigurationFile.getParent();
        if (parent == null) {
            printAndLog("Could not access scenario directory of configuration file " + scenarioConfigurationFile);
            throw new ExecutionException();
        }
        return parent;
    }

    protected CHosts loadHostsConfiguration(Path hostConfigurationFile) {
        final CHosts hostConfiguration = loadJsonConfiguration(hostConfigurationFile, new CHosts());
        if (hostConfiguration.localHosts.isEmpty()) {
            hostConfiguration.addDefaultLocalHost();
        }
        return hostConfiguration;
    }

    private <T> T loadJsonConfiguration(Path configurationFile, Class<T> clazz) throws ExecutionException {
        try (InputStream inputStream = loadResource(configurationFile)) {
            return new ObjectInstantiation<>(clazz).read(inputStream);
        } catch (IOException | InstantiationException e) {
            printAndLog("Could not load file due to invalid format.", e);
            throw new ExecutionException();
        }
    }

    private <T> T loadJsonConfiguration(Path configurationFile, T defaultObject) {
        try (InputStream inputStream = loadResource(configurationFile)) {
            return new ObjectInstantiation<>((Class<T>) defaultObject.getClass()).read(inputStream);
        } catch (IOException | InstantiationException e) {
            getLogger().warn("Could not load file {}. Using default configuration.", configurationFile);
            return defaultObject;
        }
    }

    private Logger getLogger() {
        return LoggerFactory.getLogger("MosaicStarter");
    }

    protected CRuntime loadRuntimeConfiguration(Path configurationPath) throws ExecutionException {
        try {
            return loadJsonConfiguration(configurationPath, CRuntime.class);
        } catch (Exception e) {
            printAndLog("Could not read configuration file " + configurationPath, e);
            throw new ExecutionException();
        }
    }

    private InputStream loadResource(Path path) throws FileNotFoundException {
        // if resources could not found in working dir, then search in other resources (jars and build directories)
        if (!Files.isReadable(path)) {
            InputStream resourceStream = this.getClass().getClassLoader().getResourceAsStream(path.toString());
            if (resourceStream != null) {
                return resourceStream;
            }
        }
        return new FileInputStream(path.toFile());
    }

    protected ClassLoader createClassLoader() {
        return MosaicClassLoader.includeJarFiles(LIBRARIES);
    }

    protected MosaicSimulation.ComponentProviderFactory createComponentProviderFactory(
            CRuntime runtimeConfiguration, ClassLoader classLoader
    ) throws ExecutionException {
        return MosaicComponentProvider::new;
    }

    /**
     * Print command line related properties and exit the Eclipse MOSAIC in case
     * of a invalid command line argument.
     *
     * @param cli command line into a parameter object.
     */
    protected void printUsage(CommandLineParser<?> cli, String error) {
        final ByteArrayOutputStream baos = new ByteArrayOutputStream();
        try (PrintWriter writer = new PrintWriter(new OutputStreamWriter(baos, StandardCharsets.UTF_8))) {
            cli.printHelp(writer);
        }
        String usage = new String(baos.toByteArray(), StandardCharsets.UTF_8);
        System.out.format("Invalid command line arguments: %s%n%s", error, usage);
    }

    /**
     * Log the error string and exit Eclipse MOSAIC.
     *
     * @param message a string representing the error message.
     */
    protected void printAndLog(String message) {
        System.err.println(message);
        if (getLogger() != null) {
            getLogger().error(message);
        }
    }

    /**
     * Log the error string and exit Eclipse MOSAIC.
     *
     * @param string a string representing the error message.
     * @param reason the exception which led to the error message
     */
    protected void printAndLog(String string, Throwable reason) {
        System.err.println(string);
        System.err.println("   Stacktrace: " + ExceptionUtils.getStackTrace(reason));
        if (getLogger() != null) {
            getLogger().error(string, reason);
        }
    }

    /**
     * Print the start time, end time and the duration of the simulation
     * in case of successful simulation.
     *
     * @param durationMs the duration of the simulation in milliseconds
     */
    private void printSuccessfulSimulation(Logger logger, long durationMs) {
        long endTime = System.currentTimeMillis();
        SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
        Date startDate = new Date(endTime - durationMs);
        Date endDate = new Date(endTime);
        logger.info("Simulation started: " + sdf.format(startDate));
        logger.info("Simulation ended: " + sdf.format(endDate));
        logger.info("Finishing simulation (duration: {})", DurationFormatUtils.formatDuration(durationMs, "HH'h' mm'm' ss.SSS's'"));
        System.out.println();
        System.out.println();
    }

    private void printErrorInformation(Logger logger, Throwable exception) {
        String logDirectory = null;
        try {
            logDirectory = ((LoggerContext) LoggerFactory.getILoggerFactory()).getProperty("logDirectory");
        } catch (Throwable ex) {
            System.err.println("Could not initialize logging");
            System.err.println(ExceptionUtils.getStackTrace(exception));
        }

        System.err.println();
        System.err.println("-------------------------------------------------------------------");
        System.err.println(" Stopping simulation due to a critical error:");
        System.err.println("\t- " + ExceptionUtils.getMessage(exception));
        System.err.println("\t- Root Cause: " + ExceptionUtils.getRootCauseMessage(exception));
        if (logDirectory != null) {
            System.err.println(" Please see the log files for details.");
            System.err.println("\t- Log-Directory: " + logDirectory);
        }
        System.err.println();
        System.err.println(" Eclipse MOSAIC will now shutdown.");
        System.err.println("-------------------------------------------------------------------");
        System.err.println();

        if (logger == null || logger.isDebugEnabled()) {
            System.err.println("\t- Detailed Stacktrace: ");
            System.err.println();
            System.err.println(ExceptionUtils.getStackTrace(exception));
            System.err.println();
            System.err.println("-------------------------------------------------------------------");
        }

        if (logger != null) {
            logger.error("Stopping simulation due to a critical error.", exception);
        }
    }

    private void startVisualizerInBrowser() {
        try {
            Desktop.getDesktop().open(VISUALIZER_PATH.toFile());
        } catch (IOException e) {
            getLogger().warn("Could not start visualization", e);
        } catch (UnsupportedOperationException e) {
            System.err.println();
            System.err.println("Could not open " + VISUALIZER_PATH + " in the browser.");
            System.err.println("The Desktop API might not be supported on the current system.");
            if (SystemUtils.IS_OS_UNIX) {
                System.err.println("On Linux, installing libgnome2-0 should fix this problem.");
            }
            System.err.println();
        }
    }

    public static class ExecutionException extends Exception {
        private static final long serialVersionUID = 1L;
        // marker exception
    }

}
